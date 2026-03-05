"""
本地验证脚本 — 打开 building_scene.xml (或其他场景)

用法:
    python sim/scripts/view_scene.py                      # 默认: building_scene.xml
    python sim/scripts/view_scene.py open_field           # open_field.xml
    python sim/scripts/view_scene.py building_scene       # building_scene.xml

依赖: pip install mujoco
"""
import sys
import os
import mujoco
import mujoco.viewer

# 场景选择
scene_name = sys.argv[1] if len(sys.argv) > 1 else 'building_scene'
if not scene_name.endswith('.xml'):
    scene_name += '.xml'

worlds_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'worlds')
scene_path = os.path.normpath(os.path.join(worlds_dir, scene_name))

if not os.path.exists(scene_path):
    print(f'[ERROR] 场景文件不存在: {scene_path}')
    print(f'可用场景:')
    for f in os.listdir(worlds_dir):
        if f.endswith('.xml'):
            print(f'  {f}')
    sys.exit(1)

print(f'[view_scene] 加载: {scene_path}')
model = mujoco.MjModel.from_xml_path(scene_path)
data  = mujoco.MjData(model)

print(f'  模型: {model.nbody} bodies, {model.ngeom} geoms, {model.njnt} joints')

# 打印机器人起始位置
base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'base_link')
if base_id >= 0:
    mujoco.mj_kinematics(model, data)
    pos = data.xpos[base_id]
    print(f'  机器人起始位置: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})')

print()
print('操作提示:')
print('  鼠标左键拖动  — 旋转视角')
print('  鼠标右键拖动  — 平移')
print('  滚轮          — 缩放')
print('  空格          — 暂停/继续仿真')
print('  Ctrl+A        — 重置位姿')
print('  关闭窗口       — 退出')
print()

with mujoco.viewer.launch_passive(model, data) as viewer:
    # 根据场景自动选择合适的初始视角
    if 'factory' in scene_name:
        viewer.cam.azimuth   = 220    # 斜侧视角
        viewer.cam.elevation = -20
        viewer.cam.distance  = 90     # 工厂很大，需要拉远
        viewer.cam.lookat[:] = [40, 27, 4]   # 看向工厂中心
    else:
        viewer.cam.azimuth   = 210
        viewer.cam.elevation = -25
        viewer.cam.distance  = 28
        viewer.cam.lookat[:] = [11, 7, 3]

    step = 0
    while viewer.is_running():
        mujoco.mj_step(model, data)
        if step % 60 == 0:   # 每 60 步同步一次（~0.12s 仿真时间）
            viewer.sync()
        step += 1
