"""Decision module."""

import sys
import time

sys.path.insert(0, "src")

from memory.scheduling.voi_scheduler import (
    SchedulerAction,
    SchedulerState,
    VoIConfig,
    VoIScheduler,
)


def generate_trajectory(n_steps: int = 50):
    """Generate trajectory."""
    trajectory = []
    for i in range(n_steps):
        t = i / (n_steps - 1)  # 0..1

        cred = 0.3 + 0.6 * t

        if i == 20 or i == 35:
            cred = 0.2

        dist = 10.0 - 9.5 * t

        pos_var = 2.0 - 1.9 * t

        trajectory.append(
            {
                "step": i,
                "credibility": round(cred, 4),
                "distance_to_goal": round(dist, 3),
                "position_var": round(pos_var, 4),
            }
        )
    return trajectory


def run_fixed_interval(trajectory):
    """Run fixed interval."""
    actions = []
    for step_data in trajectory:
        i = step_data["step"]
        if i > 0 and i % 5 == 0:
            actions.append(SchedulerAction.REPERCEIVE)
        else:
            actions.append(SchedulerAction.CONTINUE)
    return actions


def run_old_voi(trajectory):
    """Run old voi."""
    actions = []
    for step_data in trajectory:
        cred = step_data["credibility"]
        pos_var = step_data["position_var"]

        uncertainty = 1.0 - cred
        position_info = min(1.0, pos_var / 2.0)
        delta_s = 0.4 * uncertainty + 0.3 * position_info

        if delta_s > 0.55:
            actions.append(SchedulerAction.SLOW_REASON)
        elif delta_s > 0.35:
            actions.append(SchedulerAction.REPERCEIVE)
        else:
            actions.append(SchedulerAction.CONTINUE)
    return actions


def run_new_voi(trajectory):
    """Run new voi."""
    cfg = VoIConfig(
        reperception_cooldown=0.0,
        slow_reason_cooldown=0.0,
        min_distance_for_trigger=0.0,
    )
    scheduler = VoIScheduler(config=cfg)
    actions = []

    time.time()
    last_rep_time = 0.0
    last_slow_time = 0.0
    rep_count = 0
    slow_count = 0

    for step_data in trajectory:
        state = SchedulerState(
            target_credibility=step_data["credibility"],
            target_position_var=step_data["position_var"],
            distance_to_goal=step_data["distance_to_goal"],
            distance_since_last_reperception=2.0,
            match_count=3,
            total_objects=15,
            slow_reason_count=slow_count,
            reperception_count=rep_count,
            last_reperception_time=last_rep_time,
            last_slow_reason_time=last_slow_time,
            time_elapsed=(step_data["step"] * 2.0),
        )

        action = scheduler.decide(state)
        actions.append(action)

        now = time.time()
        if action == SchedulerAction.REPERCEIVE:
            last_rep_time = now
            rep_count += 1
        elif action == SchedulerAction.SLOW_REASON:
            last_slow_time = now
            slow_count += 1

    return actions


def evaluate(trajectory, actions):
    """Evaluate."""
    total_triggers = 0
    unnecessary = 0
    missed = 0
    slow_count = 0

    for step_data, action in zip(trajectory, actions):
        cred = step_data["credibility"]
        triggered = action != SchedulerAction.CONTINUE

        if triggered:
            total_triggers += 1
        if action == SchedulerAction.SLOW_REASON:
            slow_count += 1

        if triggered and cred > 0.7:
            unnecessary += 1

        if not triggered and cred < 0.3:
            missed += 1

    return {
        "total_triggers": total_triggers,
        "unnecessary": unnecessary,
        "missed": missed,
        "slow_reason": slow_count,
    }


def print_timeline(trajectory, results):
    """Print timeline."""
    print("\n" + "=" * 72)
    print("  50-Step Navigation Timeline")
    print("=" * 72)
    print(f"{'Step':>4}  {'Cred':>5}  {'Dist':>5}  {'Fixed':^8}  {'OldVoI':^8}  {'NewVoI':^8}")
    print("-" * 72)

    action_sym = {
        SchedulerAction.CONTINUE: "  .  ",
        SchedulerAction.REPERCEIVE: " REP ",
        SchedulerAction.SLOW_REASON: " SLO ",
    }

    for i, step_data in enumerate(trajectory):
        cred = step_data["credibility"]
        dist = step_data["distance_to_goal"]
        marker = " <<" if cred < 0.3 else ""
        print(
            f"{i:>4}  {cred:>5.2f}  {dist:>5.1f}  "
            f"{action_sym[results['FixedInterval'][i]]}  "
            f"{action_sym[results['OldVoI'][i]]}  "
            f"{action_sym[results['NewVoI'][i]]}"
            f"{marker}"
        )
    print("-" * 72)
    print("  REP = reperceive, SLO = slow_reason, · = continue, << = 误检步\n")


def print_latex_table(metrics):
    """Print latex table."""
    print("\n% ── LaTeX Table ──")
    print(r"\begin{table}[h]")
    print(r"\centering")
    print(r"\caption{VoI Scheduling Strategy Comparison (50-step navigation)}")
    print(r"\label{tab:voi-comparison}")
    print(r"\begin{tabular}{lcccc}")
    print(r"\toprule")
    print(r"Strategy & Total Triggers & Unnecessary & Missed & Slow Reason \\")
    print(r"\midrule")
    for name in ["FixedInterval", "OldVoI", "NewVoI"]:
        m = metrics[name]
        print(f"{name} & {m['total_triggers']} & {m['unnecessary']} & {m['missed']} & {m['slow_reason']} \\\\")
    print(r"\bottomrule")
    print(r"\end{tabular}")
    print(r"\end{table}")


def print_summary(metrics):
    """Print summary."""
    print("\n" + "=" * 60)
    print("  Strategy Comparison Summary")
    print("=" * 60)
    header = f"{'Strategy':<16} {'Triggers':>8} {'Unnecessary':>12} {'Missed':>8} {'SlowRsn':>8}"
    print(header)
    print("-" * 60)
    for name in ["FixedInterval", "OldVoI", "NewVoI"]:
        m = metrics[name]
        print(f"{name:<16} {m['total_triggers']:>8} {m['unnecessary']:>12} {m['missed']:>8} {m['slow_reason']:>8}")
    print("-" * 60)


def main():
    trajectory = generate_trajectory(50)

    actions_fixed = run_fixed_interval(trajectory)
    actions_old = run_old_voi(trajectory)
    actions_new = run_new_voi(trajectory)

    results = {
        "FixedInterval": actions_fixed,
        "OldVoI": actions_old,
        "NewVoI": actions_new,
    }

    metrics = {}
    for name, acts in results.items():
        metrics[name] = evaluate(trajectory, acts)

    print_timeline(trajectory, results)
    print_summary(metrics)
    print_latex_table(metrics)

    m_new = metrics["NewVoI"]
    m_fixed = metrics["FixedInterval"]

    assert m_new["missed"] == 0, f"NewVoI missed {m_new['missed']} low-cred steps"

    assert m_new["unnecessary"] <= m_fixed["unnecessary"], (
        f"NewVoI unnecessary ({m_new['unnecessary']}) > Fixed unnecessary ({m_fixed['unnecessary']})"
    )

    print("\nAll assertions passed.")


if __name__ == "__main__":
    main()
