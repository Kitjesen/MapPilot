"""
VLA Navigation — End-to-end Vision-Language-Action model for embodied navigation.

Architecture based on VLingNav (arXiv 2601.08665):
  - Backbone:    Qwen2.5-VL-3B (adapted from LLaVA-Video-7B for edge deployment)
  - AdaCoT:      Adaptive Chain-of-Thought with THINK/NO_THINK trigger
  - VLingMem:    Visual-assisted Linguistic Memory for long-horizon navigation
  - Action Head: MLP predicting continuous waypoint trajectories (x, y, theta)

Training pipeline:
  1. Habitat data collection (expert trajectories)
  2. AdaCoT annotation (rule-based + VLM labelling)
  3. SFT with LoRA on cloud GPU
  4. Online RL post-training (PPO/GRPO)

Deployment:
  - GPTQ INT4 quantization → TensorRT export → Jetson Orin NX 16GB
"""

__version__ = "0.1.0"
