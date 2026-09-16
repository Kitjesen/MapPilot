"""
Adaptive Chain-of-Thought (AdaCoT) Module.

Reference: VLingNav (arXiv 2601.08665) Section 3.3.2
  - Inspired by dual-process theory (Kahneman): System 1 (fast) / System 2 (slow)
  - Model predicts <THINK> or <NO_THINK> token at each navigation step
  - <THINK>    → generate CoT reasoning text, then predict action
  - <NO_THINK> → predict action directly (skip reasoning, ~10x faster)

Implementation:
  - Trigger head: lightweight MLP classifier on the VLM's last hidden state
  - CoT generator: VLM text generation with structured prompt
  - Training: SFT with AdaCoT annotations → RL fine-tuning with PPO
"""

import logging
from dataclasses import dataclass
from typing import Optional, Tuple

import torch
import torch.nn as nn
import torch.nn.functional as F

logger = logging.getLogger(__name__)

# Special tokens
THINK_TOKEN = "<THINK>"
NO_THINK_TOKEN = "<NO_THINK>"


@dataclass
class AdaCoTResult:
    """Output of one AdaCoT decision step."""
    should_think: bool              # True if THINK was triggered
    trigger_prob: float             # Probability of THINK (0.0 – 1.0)
    cot_text: str = ""              # Generated CoT reasoning (empty if NO_THINK)
    hidden_state: Optional[torch.Tensor] = None  # Possibly enriched hidden state


class AdaCoTTriggerHead(nn.Module):
    """
    Binary classifier: given the VLM's hidden state, predict P(THINK).

    Architecture:
        Linear(hidden_dim, 256) → GELU → Dropout → Linear(256, 2)

    During SFT, this head is trained jointly with the VLM using cross-entropy
    loss on THINK/NO_THINK labels from the Nav-AdaCoT dataset.

    During RL post-training, the trigger head's policy is optimised via PPO
    to balance reasoning quality vs. computational cost.
    """

    def __init__(self, hidden_dim: int = 2048, dropout: float = 0.1):
        super().__init__()
        self.classifier = nn.Sequential(
            nn.Linear(hidden_dim, 256),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(256, 2),  # [logit_NO_THINK, logit_THINK]
        )

    def forward(self, hidden_state: torch.Tensor) -> torch.Tensor:
        """
        Args:
            hidden_state: (batch, hidden_dim)
        Returns:
            logits: (batch, 2)  — index 0 = NO_THINK, index 1 = THINK
        """
        return self.classifier(hidden_state)

    def predict_prob(self, hidden_state: torch.Tensor) -> float:
        """Return P(THINK) as a scalar."""
        with torch.no_grad():
            logits = self.forward(hidden_state)
            probs = F.softmax(logits, dim=-1)
            return probs[0, 1].item()


class CoTGenerator:
    """
    Generate structured Chain-of-Thought reasoning text.

    The CoT output follows a fixed schema that the VLM has been trained on:
      1. Observation summary  (what the agent currently sees)
      2. Memory recall        (relevant past observations)
      3. Instruction analysis (what the instruction asks)
      4. Decision rationale   (why to go in a certain direction)
    """

    COT_PROMPT_TEMPLATE = (
        "{think_token}\n"
        "[Observation] I see: {observation}\n"
        "[Memory] Previously: {memory}\n"
        "[Instruction] Goal: {instruction}\n"
        "[Reasoning] "
    )

    def __init__(self, max_tokens: int = 128, temperature: float = 0.3):
        self.max_tokens = max_tokens
        self.temperature = temperature

    def build_prompt(
        self,
        instruction: str,
        observation_summary: str,
        memory_summary: str,
    ) -> str:
        return self.COT_PROMPT_TEMPLATE.format(
            think_token=THINK_TOKEN,
            observation=observation_summary,
            memory=memory_summary,
            instruction=instruction,
        )

    def generate(
        self,
        backbone,  # VLABackbone
        instruction: str,
        observation_summary: str,
        memory_summary: str,
        current_images: list,
    ) -> str:
        """
        Generate CoT reasoning text using the VLM backbone.

        Returns:
            Reasoning text (without the <THINK> prefix).
        """
        prompt = self.build_prompt(instruction, observation_summary, memory_summary)
        cot_text = backbone.generate_text(
            prompt=prompt,
            images=current_images,
            max_new_tokens=self.max_tokens,
            temperature=self.temperature,
        )
        return cot_text


class AdaCoTModule(nn.Module):
    """
    Full AdaCoT module: trigger decision + optional CoT generation.

    Usage:
        adacot = AdaCoTModule(hidden_dim=2048)
        result = adacot.step(hidden_state, backbone, instruction, obs, mem, images)
        if result.should_think:
            # Use result.cot_text for downstream processing
            ...
    """

    def __init__(
        self,
        hidden_dim: int = 2048,
        trigger_threshold: float = 0.5,
        max_cot_tokens: int = 128,
        cot_temperature: float = 0.3,
        dropout: float = 0.1,
    ):
        super().__init__()
        self.trigger_head = AdaCoTTriggerHead(hidden_dim, dropout)
        self.cot_generator = CoTGenerator(max_cot_tokens, cot_temperature)
        self.trigger_threshold = trigger_threshold

        # Statistics tracking
        self._total_steps: int = 0
        self._think_steps: int = 0

    @property
    def think_ratio(self) -> float:
        """Fraction of steps where THINK was triggered."""
        if self._total_steps == 0:
            return 0.0
        return self._think_steps / self._total_steps

    def forward(self, hidden_state: torch.Tensor) -> torch.Tensor:
        """
        Forward pass: return trigger logits for given hidden state.

        Enables nn.Module usage: ``adacot(x)`` → logits (batch, 2).
        Use ``step()`` for full CoT decision + generation at inference time.

        Args:
            hidden_state: (batch, hidden_dim)
        Returns:
            logits: (batch, 2)  — index 0 = NO_THINK, index 1 = THINK
        """
        return self.trigger_head(hidden_state)

    def step(
        self,
        hidden_state: torch.Tensor,
        backbone,  # VLABackbone — needed for CoT text generation
        instruction: str,
        observation_summary: str,
        memory_summary: str,
        current_images: list,
        force_think: bool = False,
    ) -> AdaCoTResult:
        """
        Execute one AdaCoT decision step.

        1. Trigger head predicts P(THINK)
        2. If P(THINK) >= threshold (or force_think): generate CoT text
        3. Return result with decision + optional CoT text

        Args:
            hidden_state: (1, hidden_dim) from VLM backbone
            backbone: VLABackbone instance for text generation
            instruction: current navigation instruction
            observation_summary: text description of current view
            memory_summary: text from VLingMem retrieval
            current_images: list of np.ndarray images for CoT prompt
            force_think: override trigger and always generate CoT

        Returns:
            AdaCoTResult with decision, probability, and optional CoT text
        """
        self._total_steps += 1

        # Trigger decision
        prob = self.trigger_head.predict_prob(hidden_state)
        should_think = force_think or (prob >= self.trigger_threshold)

        cot_text = ""
        if should_think:
            self._think_steps += 1
            cot_text = self.cot_generator.generate(
                backbone=backbone,
                instruction=instruction,
                observation_summary=observation_summary,
                memory_summary=memory_summary,
                current_images=current_images,
            )
            logger.debug(
                "AdaCoT THINK (p=%.3f): %s",
                prob,
                cot_text[:80] + ("..." if len(cot_text) > 80 else ""),
            )
        else:
            logger.debug("AdaCoT NO_THINK (p=%.3f)", prob)

        return AdaCoTResult(
            should_think=should_think,
            trigger_prob=prob,
            cot_text=cot_text,
            hidden_state=hidden_state,
        )

    def compute_trigger_loss(
        self,
        hidden_states: torch.Tensor,
        labels: torch.Tensor,
    ) -> torch.Tensor:
        """
        Compute cross-entropy loss for trigger head training.

        Args:
            hidden_states: (batch, hidden_dim)
            labels: (batch,) with values 0 (NO_THINK) or 1 (THINK)

        Returns:
            Scalar loss
        """
        logits = self.trigger_head(hidden_states)
        return F.cross_entropy(logits, labels)

    def get_statistics(self) -> dict:
        return {
            "total_steps": self._total_steps,
            "think_steps": self._think_steps,
            "think_ratio": self.think_ratio,
        }

    def reset_statistics(self):
        self._total_steps = 0
        self._think_steps = 0
