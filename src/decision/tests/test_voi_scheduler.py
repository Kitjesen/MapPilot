"""Decision module."""

import math
import time
import unittest

from memory.scheduling.voi_scheduler import (
    SchedulerAction,
    SchedulerState,
    VoIConfig,
    VoIScheduler,
)


class TestSchedulerActionEnum(unittest.TestCase):
    def test_values(self):
        self.assertEqual(SchedulerAction.CONTINUE.value, "continue")
        self.assertEqual(SchedulerAction.REPERCEIVE.value, "reperceive")
        self.assertEqual(SchedulerAction.SLOW_REASON.value, "slow_reason")


class TestShannonEntropy(unittest.TestCase):
    """Test Shannon Entropy."""

    def test_entropy_at_half(self):
        """Test entropy at half."""
        self.assertAlmostEqual(VoIScheduler._binary_entropy(0.5), 1.0, places=6)

    def test_entropy_at_zero(self):
        """Test entropy at zero."""
        self.assertEqual(VoIScheduler._binary_entropy(0.0), 0.0)

    def test_entropy_at_one(self):
        """Test entropy at one."""
        self.assertEqual(VoIScheduler._binary_entropy(1.0), 0.0)

    def test_entropy_symmetry(self):
        """Test entropy symmetry."""
        for p in [0.1, 0.2, 0.3, 0.4]:
            self.assertAlmostEqual(
                VoIScheduler._binary_entropy(p),
                VoIScheduler._binary_entropy(1.0 - p),
                places=10,
            )

    def test_entropy_monotone_increase_to_half(self):
        """Test entropy monotone increase to half."""
        prev = 0.0
        for p in [0.1, 0.2, 0.3, 0.4, 0.5]:
            h = VoIScheduler._binary_entropy(p)
            self.assertGreaterEqual(h, prev)
            prev = h

    def test_entropy_known_value(self):
        """Test entropy known value."""
        expected = -(0.25 * math.log2(0.25) + 0.75 * math.log2(0.75))
        self.assertAlmostEqual(VoIScheduler._binary_entropy(0.25), expected, places=4)


class TestBetaVariance(unittest.TestCase):
    """Test Beta Variance."""

    def test_uniform_prior(self):
        """Test uniform prior."""
        self.assertAlmostEqual(VoIScheduler._beta_variance(1.0, 1.0), 1.0 / 12.0, places=4)

    def test_concentrated(self):
        """Test concentrated."""
        var_weak = VoIScheduler._beta_variance(1.0, 1.0)
        var_strong = VoIScheduler._beta_variance(10.0, 10.0)
        self.assertLess(var_strong, var_weak)

    def test_zero_params(self):
        """Test zero params."""
        self.assertEqual(VoIScheduler._beta_variance(0.0, 0.0), 0.25)


class TestBeliefResolution(unittest.TestCase):
    """Test Belief Resolution."""

    def setUp(self):
        self.scheduler = VoIScheduler()

    def test_explicit_alpha_beta(self):
        """Test explicit alpha beta."""
        state = SchedulerState(belief_alpha=3.0, belief_beta=7.0)
        alpha, beta, p = self.scheduler._resolve_belief(state)
        self.assertEqual(alpha, 3.0)
        self.assertEqual(beta, 7.0)
        self.assertAlmostEqual(p, 0.3, places=6)

    def test_derived_from_credibility(self):
        """Test derived from credibility."""
        state = SchedulerState(target_credibility=0.7)
        alpha, beta, p = self.scheduler._resolve_belief(state)
        self.assertAlmostEqual(p, 0.7, places=2)
        self.assertAlmostEqual(alpha + beta, 4.0, places=2)  # default concentration

    def test_credibility_zero_clamps(self):
        """Test credibility zero clamps."""
        state = SchedulerState(target_credibility=0.0)
        alpha, beta, _p = self.scheduler._resolve_belief(state)
        self.assertGreater(alpha, 0.0)
        self.assertGreater(beta, 0.0)


class TestInformationGain(unittest.TestCase):
    """Test Information Gain."""

    def setUp(self):
        self.scheduler = VoIScheduler()

    def test_gain_non_negative(self):
        """Test gain non negative."""
        for alpha, beta in [(2, 2), (1, 1), (10, 3), (0.5, 0.5)]:
            gain = self.scheduler._info_gain(alpha, beta, 3.0)
            self.assertGreaterEqual(gain, 0.0)

    def test_stronger_obs_more_gain(self):
        """Test stronger obs more gain."""
        alpha, beta = 2.0, 2.0
        gain_weak = self.scheduler._info_gain(alpha, beta, 1.0)
        gain_strong = self.scheduler._info_gain(alpha, beta, 10.0)
        self.assertGreater(gain_strong, gain_weak)

    def test_certain_belief_near_zero_gain(self):
        """Test certain belief near zero gain."""
        gain = self.scheduler._info_gain(100.0, 1.0, 5.0)
        self.assertLess(gain, 0.1)

    def test_uncertain_belief_large_gain(self):
        """Test uncertain belief large gain."""
        gain = self.scheduler._info_gain(1.0, 1.0, 5.0)
        self.assertGreater(gain, 0.1)

    def test_weak_prior_more_informative(self):
        """Test weak prior more informative."""
        gain_weak = self.scheduler._info_gain(1.0, 1.0, 3.0)
        gain_strong = self.scheduler._info_gain(10.0, 10.0, 3.0)
        self.assertGreater(gain_weak, gain_strong)


class TestVoISchedulerHardConstraints(unittest.TestCase):
    def setUp(self):
        self.scheduler = VoIScheduler()

    def _state(self, **kwargs) -> SchedulerState:
        """Helper: SchedulerState with safe defaults then override."""
        defaults = dict(
            target_credibility=0.5,
            target_existence_prob=0.6,
            target_position_var=1.0,
            belief_alpha=0.0,
            belief_beta=0.0,
            match_count=0,
            total_objects=5,
            distance_to_goal=5.0,
            nav_accumulated_dist=0.0,
            distance_since_last_reperception=2.0,  # above min_distance
            slow_reason_count=0,
            reperception_count=0,
            time_elapsed=0.0,
            last_reperception_time=0.0,
            last_slow_reason_time=0.0,
        )
        defaults.update(kwargs)
        return SchedulerState(**defaults)

    def test_danger_threshold_forces_reperceive(self):
        """Test danger threshold forces reperceive."""
        state = self._state(
            target_credibility=0.1,
            last_reperception_time=0.0,
        )
        action = self.scheduler.decide(state)
        self.assertEqual(action, SchedulerAction.REPERCEIVE)

    def test_danger_threshold_blocked_by_cooldown(self):
        """Test danger threshold blocked by cooldown."""
        state = self._state(
            target_credibility=0.1,
            last_reperception_time=time.time(),
        )
        action = self.scheduler.decide(state)
        self.assertNotEqual(action, SchedulerAction.REPERCEIVE)

    def test_safe_and_close_forces_continue(self):
        """Test safe and close forces continue."""
        state = self._state(
            target_credibility=0.9,
            distance_to_goal=1.5,
        )
        action = self.scheduler.decide(state)
        self.assertEqual(action, SchedulerAction.CONTINUE)

    def test_high_credibility_but_far_does_not_force_continue(self):
        """Test high credibility but far does not force continue."""
        state = self._state(
            target_credibility=0.9,
            distance_to_goal=10.0,
        )
        action = self.scheduler.decide(state)
        self.assertIsInstance(action, SchedulerAction)

    def test_insufficient_movement_forces_continue(self):
        """Test insufficient movement forces continue."""
        state = self._state(
            target_credibility=0.5,
            distance_since_last_reperception=0.1,
        )
        action = self.scheduler.decide(state)
        self.assertEqual(action, SchedulerAction.CONTINUE)


class TestVoISchedulerUtilityPaths(unittest.TestCase):
    def setUp(self):
        self.scheduler = VoIScheduler()

    def _state(self, **kwargs) -> SchedulerState:
        defaults = dict(
            target_credibility=0.5,
            target_existence_prob=0.5,
            target_position_var=1.0,
            belief_alpha=0.0,
            belief_beta=0.0,
            match_count=0,
            total_objects=10,
            distance_to_goal=10.0,
            nav_accumulated_dist=5.0,
            distance_since_last_reperception=3.0,
            slow_reason_count=0,
            reperception_count=0,
            time_elapsed=30.0,
            last_reperception_time=0.0,
            last_slow_reason_time=0.0,
        )
        defaults.update(kwargs)
        return SchedulerState(**defaults)

    def test_returns_valid_action(self):
        state = self._state()
        action = self.scheduler.decide(state)
        self.assertIn(action, list(SchedulerAction))

    def test_no_matches_favors_reperception(self):
        """match_count=0 adds bonus to reperceive utility."""
        state_no_match = self._state(match_count=0, target_credibility=0.45)
        state_with_match = self._state(match_count=5, target_credibility=0.45)
        u_no_match = self.scheduler._utility_reperceive(state_no_match)
        u_with_match = self.scheduler._utility_reperceive(state_with_match)
        self.assertGreater(u_no_match, u_with_match)
        action = self.scheduler.decide(state_no_match)
        self.assertIsInstance(action, SchedulerAction)

    def test_slow_reason_cooldown_blocks_slow(self):
        """If slow_reason on cooldown, SLOW_REASON utility set to -999."""
        state = self._state(
            last_slow_reason_time=time.time(),
        )
        action = self.scheduler.decide(state)
        self.assertNotEqual(action, SchedulerAction.SLOW_REASON)

    def test_reperceive_cooldown_blocks_reperceive(self):
        """If reperceive on cooldown, REPERCEIVE utility set to -999."""
        state = self._state(
            last_reperception_time=time.time(),
            target_credibility=0.5,
        )
        action = self.scheduler.decide(state)
        self.assertNotEqual(action, SchedulerAction.REPERCEIVE)

    def test_high_slow_reason_count_diminishes_slow_path(self):
        """Repeated slow reasoning reduces its marginal utility."""
        state_fresh = self._state(slow_reason_count=0)
        state_tired = self._state(slow_reason_count=20)
        action_fresh = self.scheduler.decide(state_fresh)
        action_tired = self.scheduler.decide(state_tired)
        self.assertIsInstance(action_fresh, SchedulerAction)
        self.assertIsInstance(action_tired, SchedulerAction)

    def test_complex_scene_boosts_slow_path(self):
        """Complex scene (many objects) raises slow_reason utility."""
        state = self._state(
            target_credibility=0.45,
            total_objects=50,
            match_count=5,
            distance_since_last_reperception=3.0,
        )
        action = self.scheduler.decide(state)
        self.assertIsInstance(action, SchedulerAction)

    def test_entropy_drives_utility_ordering(self):
        """Test entropy drives utility ordering."""

        cfg = VoIConfig(lambda_t=0.0, lambda_e=0.0, lambda_d=0.0)
        scheduler = VoIScheduler(config=cfg)
        state = self._state(
            target_credibility=0.5,
            match_count=0,
            total_objects=15,
        )
        u_cont = scheduler._utility_continue(state)
        u_rep = scheduler._utility_reperceive(state)
        u_slow = scheduler._utility_slow_reason(state)

        self.assertGreater(u_rep, u_cont)
        self.assertGreater(u_slow, u_cont)

    def test_low_entropy_favors_continue(self):
        """Test low entropy favors continue."""
        state = self._state(
            target_credibility=0.95,
            match_count=5,
            distance_to_goal=10.0,
        )
        u_cont = self.scheduler._utility_continue(state)
        u_rep = self.scheduler._utility_reperceive(state)
        u_slow = self.scheduler._utility_slow_reason(state)

        self.assertGreater(u_cont, u_rep)
        self.assertGreater(u_cont, u_slow)

    def test_beta_params_affect_utility(self):
        """Test beta params affect utility."""
        state_weak = self._state(belief_alpha=1.0, belief_beta=1.0)  # weak prior
        state_strong = self._state(belief_alpha=10.0, belief_beta=10.0)  # strong prior
        u_rep_weak = self.scheduler._utility_reperceive(state_weak)
        u_rep_strong = self.scheduler._utility_reperceive(state_strong)

        self.assertGreater(u_rep_weak, u_rep_strong)


class TestVoISchedulerDecisionLog(unittest.TestCase):
    def setUp(self):
        self.scheduler = VoIScheduler()

    def _state(self, credibility=0.5) -> SchedulerState:
        return SchedulerState(
            target_credibility=credibility,
            distance_to_goal=5.0,
            distance_since_last_reperception=2.0,
            last_reperception_time=0.0,
            last_slow_reason_time=0.0,
        )

    def test_log_grows_per_decision(self):
        self.assertEqual(len(self.scheduler._decision_log), 0)
        self.scheduler.decide(self._state())
        self.assertEqual(len(self.scheduler._decision_log), 1)
        self.scheduler.decide(self._state())
        self.assertEqual(len(self.scheduler._decision_log), 2)

    def test_log_capped_at_50(self):
        for _ in range(60):
            self.scheduler.decide(self._state())
        self.assertLessEqual(len(self.scheduler._decision_log), 50)

    def test_log_entry_fields(self):
        self.scheduler.decide(self._state())
        entry = self.scheduler._decision_log[-1]
        self.assertIn("time", entry)
        self.assertIn("action", entry)
        self.assertIn("cred", entry)
        self.assertIn("entropy_bits", entry)
        self.assertIn("reason", entry)

    def test_log_entropy_bits_range(self):
        """Test log entropy bits range."""
        self.scheduler.decide(self._state(credibility=0.5))
        entry = self.scheduler._decision_log[-1]
        self.assertGreaterEqual(entry["entropy_bits"], 0.0)
        self.assertLessEqual(entry["entropy_bits"], 1.0 + 1e-6)

    def test_decision_stats_counts(self):
        # Force CONTINUE (safe + close)
        self.scheduler.decide(
            SchedulerState(
                target_credibility=0.9,
                distance_to_goal=1.0,
                distance_since_last_reperception=2.0,
                last_reperception_time=0.0,
                last_slow_reason_time=0.0,
            )
        )
        # Force REPERCEIVE (danger threshold)
        self.scheduler.decide(
            SchedulerState(
                target_credibility=0.1,
                distance_to_goal=5.0,
                distance_since_last_reperception=2.0,
                last_reperception_time=0.0,
                last_slow_reason_time=0.0,
            )
        )
        stats = self.scheduler.decision_stats
        self.assertIn("continue", stats)
        self.assertIn("reperceive", stats)
        self.assertIn("slow_reason", stats)
        self.assertGreaterEqual(stats["continue"] + stats["reperceive"], 2)


class TestVoIConfig(unittest.TestCase):
    def test_custom_config(self):
        cfg = VoIConfig(
            lambda_t=0.5,
            credibility_safe=0.8,
            credibility_danger=0.2,
        )
        scheduler = VoIScheduler(config=cfg)
        state = SchedulerState(
            target_credibility=0.1,
            distance_to_goal=5.0,
            distance_since_last_reperception=2.0,
            last_reperception_time=0.0,
            last_slow_reason_time=0.0,
        )
        action = scheduler.decide(state)
        self.assertEqual(action, SchedulerAction.REPERCEIVE)

    def test_default_config_instantiation(self):
        scheduler = VoIScheduler()
        self.assertIsNotNone(scheduler._config)
        self.assertEqual(scheduler._config.lambda_t, 0.3)

    def test_zero_cost_weights(self):
        """With zero cost weights all actions have equal cost structure."""
        cfg = VoIConfig(lambda_t=0.0, lambda_e=0.0, lambda_d=0.0)
        scheduler = VoIScheduler(config=cfg)
        state = SchedulerState(
            target_credibility=0.5,
            distance_to_goal=5.0,
            distance_since_last_reperception=2.0,
            last_reperception_time=0.0,
            last_slow_reason_time=0.0,
        )
        action = scheduler.decide(state)
        self.assertIsInstance(action, SchedulerAction)

    def test_custom_obs_strength(self):
        """Test custom obs strength."""
        cfg_strong = VoIConfig(reperceive_obs_strength=10.0)
        cfg_weak = VoIConfig(reperceive_obs_strength=0.5)
        sched_strong = VoIScheduler(config=cfg_strong)
        sched_weak = VoIScheduler(config=cfg_weak)
        state = SchedulerState(
            target_credibility=0.5,
            distance_to_goal=5.0,
            distance_since_last_reperception=2.0,
            last_reperception_time=0.0,
            last_slow_reason_time=0.0,
        )
        u_strong = sched_strong._utility_reperceive(state)
        u_weak = sched_weak._utility_reperceive(state)
        self.assertGreater(u_strong, u_weak)


class TestVoIUtilityInternals(unittest.TestCase):
    def setUp(self):
        self.scheduler = VoIScheduler()

    def test_utility_continue_positive_with_uncertainty(self):
        state = SchedulerState(
            target_credibility=0.0,
            distance_to_goal=5.0,
            distance_since_last_reperception=0.0,
            last_reperception_time=0.0,
            last_slow_reason_time=0.0,
        )
        u = self.scheduler._utility_continue(state)
        self.assertIsInstance(u, float)

    def test_utility_reperceive_bonus_for_no_match(self):
        state_no_match = SchedulerState(
            target_credibility=0.5,
            match_count=0,
            target_position_var=1.0,
            distance_to_goal=5.0,
            distance_since_last_reperception=0.0,
            last_reperception_time=0.0,
            last_slow_reason_time=0.0,
        )
        state_with_match = SchedulerState(
            target_credibility=0.5,
            match_count=5,
            target_position_var=1.0,
            distance_to_goal=5.0,
            distance_since_last_reperception=0.0,
            last_reperception_time=0.0,
            last_slow_reason_time=0.0,
        )
        u_no_match = self.scheduler._utility_reperceive(state_no_match)
        u_with_match = self.scheduler._utility_reperceive(state_with_match)
        self.assertGreater(u_no_match, u_with_match)

    def test_utility_slow_reason_diminishes(self):
        base = SchedulerState(
            target_credibility=0.5,
            total_objects=10,
            slow_reason_count=0,
            distance_to_goal=5.0,
            distance_since_last_reperception=0.0,
            last_reperception_time=0.0,
            last_slow_reason_time=0.0,
        )
        many = SchedulerState(
            target_credibility=0.5,
            total_objects=10,
            slow_reason_count=10,
            distance_to_goal=5.0,
            distance_since_last_reperception=0.0,
            last_reperception_time=0.0,
            last_slow_reason_time=0.0,
        )
        u_base = self.scheduler._utility_slow_reason(base)
        u_many = self.scheduler._utility_slow_reason(many)
        self.assertGreater(u_base, u_many)

    def test_utility_reperceive_position_var_effect(self):
        """Test utility reperceive position var effect."""
        state_low_var = SchedulerState(
            target_credibility=0.5,
            target_position_var=0.1,
            match_count=2,
            distance_to_goal=5.0,
            distance_since_last_reperception=0.0,
            last_reperception_time=0.0,
            last_slow_reason_time=0.0,
        )
        state_high_var = SchedulerState(
            target_credibility=0.5,
            target_position_var=5.0,
            match_count=2,
            distance_to_goal=5.0,
            distance_since_last_reperception=0.0,
            last_reperception_time=0.0,
            last_slow_reason_time=0.0,
        )
        u_low = self.scheduler._utility_reperceive(state_low_var)
        u_high = self.scheduler._utility_reperceive(state_high_var)
        self.assertGreater(u_high, u_low)


if __name__ == "__main__":
    unittest.main()
