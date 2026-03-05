#!/usr/bin/env python3
"""
MapPilot MuJoCo Simulation — 主入口

用法：
    # 平地测试（快速验证 LiDAR + bridge）
    python sim/scripts/run_sim.py --world open_field

    # 螺旋地形（需先运行 gen_terrain_mesh.py）
    python sim/scripts/run_sim.py --world spiral_terrain

    # 无 GUI（headless，用于 CI / 服务器）
    python sim/scripts/run_sim.py --headless

    # 不启动 ROS2（纯 MuJoCo 调试）
    python sim/scripts/run_sim.py --no-ros
"""
import sys
import os
import argparse
import time
import numpy as np

# sim/ 目录加入 path
SIM_DIR  = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJ_DIR = os.path.dirname(SIM_DIR)
sys.path.insert(0, SIM_DIR)
sys.path.insert(0, PROJ_DIR)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--world',    default='open_field',
                   choices=['open_field', 'spiral_terrain'])
    p.add_argument('--headless', action='store_true',
                   help='no GUI viewer')
    p.add_argument('--no-ros',   action='store_true',
                   help='skip ROS2 bridge')
    p.add_argument('--lidar-hz', type=float, default=10.0,
                   help='LiDAR publish frequency (Hz)')
    p.add_argument('--sim-speed', type=float, default=1.0,
                   help='realtime factor (1.0=realtime, 2.0=2x fast)')
    return p.parse_args()


def main():
    args = parse_args()

    # ── 加载 MuJoCo ─────────────────────────────────────────────────────────
    try:
        import mujoco
        import mujoco.viewer
    except ImportError:
        print('ERROR: mujoco not installed.')
        print('       pip install mujoco')
        sys.exit(1)

    world_path = os.path.join(SIM_DIR, 'worlds', f'{args.world}.xml')
    if not os.path.exists(world_path):
        # spiral_terrain needs meshes
        if args.world == 'spiral_terrain':
            print(f'Meshes not found. Run first:')
            print(f'  python sim/scripts/gen_terrain_mesh.py')
        print(f'ERROR: world file not found: {world_path}')
        sys.exit(1)

    print(f'[Sim] Loading world: {world_path}')
    model = mujoco.MjModel.from_xml_path(world_path)
    data  = mujoco.MjData(model)

    print(f'[Sim] Model: {model.nbody} bodies, {model.ngeom} geoms, '
          f'{model.nq} DOF, dt={model.opt.timestep*1000:.1f}ms')

    # ── LiDAR 仿真器 ─────────────────────────────────────────────────────────
    try:
        from sensors.livox_mid360 import LivoxMid360SimVectorized
        lidar = LivoxMid360SimVectorized(model, data,
                                         body_name='lidar_link',
                                         add_noise=True)
        print(f'[Sim] LiDAR: LivoxMid360 vectorized (mj_multiRay)')
    except Exception as e:
        print(f'[Sim] WARNING: LiDAR init failed: {e}')
        lidar = None

    # ── ROS2 Bridge ───────────────────────────────────────────────────────────
    bridge = None
    if not args.no_ros:
        try:
            from bridge.mujoco_ros2_bridge import MuJoCoROS2Bridge
            bridge = MuJoCoROS2Bridge(model, data, lidar=lidar,
                                       robot_body='base_link',
                                       lidar_freq=args.lidar_hz)
        except Exception as e:
            print(f'[Sim] WARNING: ROS2 bridge failed: {e}')
            bridge = None

    # ── 仿真主循环 ────────────────────────────────────────────────────────────
    if args.headless:
        _run_headless(model, data, bridge, args)
    else:
        _run_with_viewer(model, data, bridge, args)

    if bridge:
        bridge.destroy()


def _run_headless(model, data, bridge, args):
    """无 GUI 模式，用于 CI 和服务器。"""
    import mujoco
    print('[Sim] Running headless...')
    step = 0
    dt   = model.opt.timestep
    t_start = time.perf_counter()

    while True:
        mujoco.mj_step(model, data)
        step += 1

        if bridge:
            bridge.spin_once()

        # 实时速度控制
        sim_time   = data.time
        wall_time  = time.perf_counter() - t_start
        sleep_time = sim_time / args.sim_speed - wall_time
        if sleep_time > 0:
            time.sleep(sleep_time)

        if step % 5000 == 0:
            body_id = 0
            try:
                import mujoco as mj
                body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, 'base_link')
            except Exception:
                pass
            pos = data.xpos[body_id]
            print(f'[Sim] t={data.time:.1f}s  robot=({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f})')


def _run_with_viewer(model, data, bridge, args):
    """带 GUI viewer 模式。"""
    import mujoco
    import mujoco.viewer

    print('[Sim] Opening MuJoCo viewer...')
    print('      Ctrl+C or close window to exit')

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.azimuth   = -50
        viewer.cam.elevation = 20
        viewer.cam.distance  = 30.0

        t_start = time.perf_counter()
        while viewer.is_running():
            step_start = time.perf_counter()

            mujoco.mj_step(model, data)

            if bridge:
                bridge.spin_once()

            # 同步 viewer
            viewer.sync()

            # 实时控制
            dt = model.opt.timestep / args.sim_speed
            elapsed = time.perf_counter() - step_start
            if elapsed < dt:
                time.sleep(dt - elapsed)


if __name__ == '__main__':
    main()
