"""
test_implicit_fsm_policy.py — 隐式 FSM 策略测试
"""

import tempfile
import unittest
from pathlib import Path

import numpy as np

from decision.tasking.implicit_fsm_policy import (
    ImplicitFSMObservation,
    ImplicitFSMPolicy,
)


class TestImplicitFSMPolicy(unittest.TestCase):
    def _make_obs(
        self,
        predicted_object: str,
        conf: float,
        mission_state: str,
        search_state: str,
        instr_changed: bool = False,
    ) -> ImplicitFSMObservation:
        inst0 = "go to door"
        inst1 = "find door" if instr_changed else inst0
        return ImplicitFSMObservation(
            mission_instruction_0=inst0,
            mission_instruction_1=inst1,
            mission_object_1="door",
            predicted_object=predicted_object,
            confidence=conf,
            object_xyn=np.array([0.5, 0.5]),
            object_whn=np.array([0.7, 0.7]),
            mission_state_in=mission_state,
            search_state_in=search_state,
        )

    def test_missing_target_prefers_searching(self):
        policy = ImplicitFSMPolicy()
        obs = self._make_obs(
            predicted_object="NULL",
            conf=0.05,
            mission_state="running",
            search_state="had_searching_1",
            instr_changed=False,
        )
        pred = policy.predict(obs)

        self.assertIsNotNone(pred)
        self.assertIn(pred.mission_state_out, {"searching_0", "searching_1"})

    def test_strong_match_can_reach_success(self):
        policy = ImplicitFSMPolicy()
        obs = self._make_obs(
            predicted_object="door",
            conf=0.95,
            mission_state="running",
            search_state="had_searching_1",
        )
        pred = policy.predict(obs)

        self.assertIsNotNone(pred)
        self.assertIn(pred.mission_state_out, {"success", "running"})
        self.assertGreaterEqual(float(np.max(pred.state_prob)), 0.3)

    def test_export_and_reload_weights(self):
        with tempfile.TemporaryDirectory() as d:
            weight_path = Path(d) / "fsm_weights.npz"
            ImplicitFSMPolicy.export_default_weights(str(weight_path))
            policy = ImplicitFSMPolicy(weights_path=str(weight_path))

            obs = self._make_obs(
                predicted_object="door",
                conf=0.8,
                mission_state="searching_1",
                search_state="had_searching_1",
            )
            pred = policy.predict(obs)

            self.assertIsNotNone(pred)
            self.assertEqual(pred.motion_vector.shape, (3,))


if __name__ == "__main__":
    unittest.main()
