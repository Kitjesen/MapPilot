#!/usr/bin/env python3
"""Deploy staircase viz to robot: live MuJoCo + offscreen render."""
import paramiko
import sys
import io
import time
import os

sys.path.insert(0, 'C:/tmp')
from factory_stairs_scene import generate_scene_xml

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

SCENE_XML = generate_scene_xml([10.2, 10.0, 0.6])

LIVE_VIZ = '''#!/usr/bin/env python3
"""Factory 3-floor staircase MuJoCo viz — live on robot screen."""
import mujoco
import mujoco.viewer
import numpy as np
import time
import traceback

path_arr = np.load("/tmp/factory_global_path.npy")
# Downsample path to ~80 waypoints for smoother viz
skip = max(1, len(path_arr) // 80)
path_ds = path_arr[::skip].tolist()
if not np.allclose(path_arr[-1], path_ds[-1]):
    path_ds.append(path_arr[-1].tolist())
goal_pos = path_ds[-1]
print(f"Start: ({path_ds[0][0]:.1f},{path_ds[0][1]:.1f},{path_ds[0][2]:.1f})")
print(f"  Goal: ({goal_pos[0]:.1f},{goal_pos[1]:.1f},{goal_pos[2]:.1f})")
print(f"  Path: {len(path_arr)} wp -> {len(path_ds)} ds, Z=[{path_arr[:,2].min():.1f},{path_arr[:,2].max():.1f}]")

SCENE_XML = """''' + SCENE_XML.replace('\\', '\\\\').replace('"""', '\\"\\"\\"') + '''"""

model = mujoco.MjModel.from_xml_string(SCENE_XML)
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

qadr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "robot_jnt")]
data.qpos[qadr] = path_ds[0][0]
data.qpos[qadr+1] = path_ds[0][1]
data.qpos[qadr+2] = path_ds[0][2]
data.qpos[qadr+3] = 1.0
mujoco.mj_forward(model, data)

# Pre-compute eye3 flat (reuse every frame)
EYE3 = np.eye(3).flatten().astype(np.float64)

trail = []
MAX_TRAIL = 150
wp_idx = 0
goal_reached = False
speed = 2.5
RENDER_DT = 0.033  # 30 fps

print("  Path=colored spheres | Blue=trail | Cyan=wp | Red=goal")
print("=" * 60)

try:
    with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as viewer:
        viewer.cam.lookat[:] = [15, 8, 3.0]
        viewer.cam.distance = 35
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -22

        t_start = time.time()
        last_log = 0

        while viewer.is_running() and not goal_reached:
            t_now = time.time() - t_start
            pos = [data.qpos[qadr], data.qpos[qadr+1], data.qpos[qadr+2]]
            wp = path_ds[wp_idx]
            dx, dy, dz = wp[0]-pos[0], wp[1]-pos[1], wp[2]-pos[2]
            dist_3d = np.sqrt(dx**2 + dy**2 + dz**2)

            # Advance waypoint
            while dist_3d < 0.8 and wp_idx < len(path_ds)-1:
                wp_idx += 1
                wp = path_ds[wp_idx]
                dx, dy, dz = wp[0]-pos[0], wp[1]-pos[1], wp[2]-pos[2]
                dist_3d = np.sqrt(dx**2 + dy**2 + dz**2)

            if dist_3d > 0.01:
                move = min(speed * RENDER_DT, dist_3d)
                nx, ny, nz = dx/dist_3d, dy/dist_3d, dz/dist_3d
                data.qpos[qadr] += nx * move
                data.qpos[qadr+1] += ny * move
                data.qpos[qadr+2] += nz * move
                yaw = np.arctan2(dy, dx)
                data.qpos[qadr+3] = np.cos(yaw/2)
                data.qpos[qadr+6] = np.sin(yaw/2)

            data.qvel[:] = 0
            mujoco.mj_forward(model, data)

            # Trail (capped)
            trail.append([pos[0], pos[1], pos[2]])
            if len(trail) > MAX_TRAIL:
                trail = trail[-(MAX_TRAIL):]

            gd = np.sqrt((pos[0]-goal_pos[0])**2+(pos[1]-goal_pos[1])**2+(pos[2]-goal_pos[2])**2)
            if wp_idx >= len(path_ds)-1 and gd < 1.5:
                goal_reached = True
                print(f"\\n*** GOAL REACHED at ({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f}) in {t_now:.1f}s ***")

            if t_now - last_log >= 1.5:
                last_log = t_now
                print(f"t={t_now:.1f}s pos=({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f}) wp={wp_idx}/{len(path_ds)} z={pos[2]:.2f} dist={gd:.1f}m")

            # Draw user_scn (path + trail + markers)
            scn = viewer.user_scn
            scn.ngeom = 0
            z_min, z_max = path_arr[:,2].min(), path_arr[:,2].max()

            # Path spheres (every 3rd downsampled point)
            for i in range(0, len(path_ds), 3):
                if scn.ngeom >= 280: break
                p = path_ds[i]
                t = (p[2]-z_min)/(z_max-z_min+1e-6)
                if t < 0.25: r,g,b = 0,t*4,1
                elif t < 0.5: r,g,b = 0,1,1-(t-0.25)*4
                elif t < 0.75: r,g,b = (t-0.5)*4,1,0
                else: r,g,b = 1,1-(t-0.75)*4,0
                mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                    np.array([0.1,0,0]), np.array([p[0],p[1],p[2]+0.15],dtype=np.float64),
                    EYE3, np.array([r,g,b,0.7],dtype=np.float32))
                scn.ngeom += 1

            # Trail (blue, every other point)
            for i in range(0, len(trail), 2):
                if scn.ngeom >= 380: break
                p = trail[i]
                mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                    np.array([0.07,0,0]), np.array([p[0],p[1],p[2]+0.25],dtype=np.float64),
                    EYE3, np.array([0.2,0.5,1,0.8],dtype=np.float32))
                scn.ngeom += 1

            # Current waypoint (cyan)
            if wp_idx < len(path_ds) and scn.ngeom < 395:
                wp2 = path_ds[wp_idx]
                mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                    np.array([0.2,0,0]), np.array([wp2[0],wp2[1],wp2[2]+0.3],dtype=np.float64),
                    EYE3, np.array([0,1,1,0.9],dtype=np.float32))
                scn.ngeom += 1

            # Goal (red)
            if scn.ngeom < 398:
                mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                    np.array([0.35,0,0]),
                    np.array([goal_pos[0],goal_pos[1],goal_pos[2]+0.4],dtype=np.float64),
                    EYE3, np.array([1,0,0,0.85],dtype=np.float32))
                scn.ngeom += 1

            viewer.sync()
            time.sleep(RENDER_DT)

        print("\\nDone. Saving trajectory...")
        np.save("/tmp/factory_traj.npy", np.array(trail))

        # Keep viewer open for 10s
        for _ in range(200):
            if not viewer.is_running(): break
            viewer.sync()
            time.sleep(0.05)

except Exception as e:
    print(f"ERROR: {e}")
    traceback.print_exc()
    if trail:
        np.save("/tmp/factory_traj.npy", np.array(trail))

print("Exit.")
'''

RENDER_SCRIPT = '''#!/usr/bin/env python3
"""Offscreen render factory staircase scene with path."""
import mujoco
import numpy as np
from PIL import Image, ImageDraw

path = np.load("/tmp/factory_global_path.npy")
print(f"Path: {path.shape}, Z=[{path[:,2].min():.1f}, {path[:,2].max():.1f}]")

SCENE_XML = """''' + SCENE_XML.replace('\\', '\\\\').replace('"""', '\\"\\"\\"') + '''"""

model = mujoco.MjModel.from_xml_string(SCENE_XML)
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
print("Scene loaded")

renderer = mujoco.Renderer(model, height=1080, width=1920)
scene_option = mujoco.MjvOption()

def add_path(scn):
    z_min, z_max = path[:,2].min(), path[:,2].max()
    count = 0
    for i in range(len(path)):
        if scn.ngeom >= scn.maxgeom - 5: break
        p = path[i]
        t = (p[2]-z_min)/(z_max-z_min+1e-6)
        if t < 0.25: r,g,b = 0,t*4,1
        elif t < 0.5: r,g,b = 0,1,1-(t-0.25)*4
        elif t < 0.75: r,g,b = (t-0.5)*4,1,0
        else: r,g,b = 1,1-(t-0.75)*4,0
        pos = p.copy(); pos[2] += 0.18
        mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([0.15,0,0]), pos.astype(np.float64),
            np.eye(3).flatten().astype(np.float64),
            np.array([r,g,b,1.0],dtype=np.float32))
        scn.ngeom += 1
        count += 1
    # Start
    if scn.ngeom < scn.maxgeom-2:
        sp = path[0].copy(); sp[2] += 0.5
        mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([0.4,0,0]), sp.astype(np.float64),
            np.eye(3).flatten().astype(np.float64),
            np.array([0,1,0,0.9],dtype=np.float32))
        scn.ngeom += 1
    # Goal
    if scn.ngeom < scn.maxgeom-2:
        gp = path[-1].copy(); gp[2] += 0.5
        mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([0.4,0,0]), gp.astype(np.float64),
            np.eye(3).flatten().astype(np.float64),
            np.array([1,0,0,0.9],dtype=np.float32))
        scn.ngeom += 1
    print(f"  {count} path pts")

def render_view(cam, filename, label):
    renderer.update_scene(data, camera=cam, scene_option=scene_option)
    add_path(renderer._scene)
    img = renderer.render()
    pil = Image.fromarray(img)
    draw = ImageDraw.Draw(pil)
    draw.rectangle([(0,0),(650,70)], fill=(0,0,0,180))
    draw.text((12,8), label, fill=(255,255,255))
    draw.text((12,30), f"Path: {len(path)} wp | Z=[{path[:,2].min():.1f},{path[:,2].max():.1f}]m | 18cm steps",
              fill=(200,200,200))
    draw.text((12,50), "Blue(1F)->Cyan->Green(2F)->Yellow->Red(3F)", fill=(180,210,255))
    pil.save(filename, quality=95)
    print(f"  Saved {filename}")

# SE overview
c1 = mujoco.MjvCamera()
c1.lookat[:] = [15,8,3]; c1.distance = 35; c1.azimuth = 135; c1.elevation = -22
print("SE overview..."); render_view(c1, "/tmp/fs_se.png", "Factory 3-Floor Stairs — SE Overview")

# East side — look from south to see into stairwells
c2 = mujoco.MjvCamera()
c2.lookat[:] = [23,7.5,3]; c2.distance = 22; c2.azimuth = 180; c2.elevation = -12
print("East side..."); render_view(c2, "/tmp/fs_east.png", "Stairwell Profile — East Side")

# NW overview
c3 = mujoco.MjvCamera()
c3.lookat[:] = [14,8,3]; c3.distance = 40; c3.azimuth = -40; c3.elevation = -30
print("NW overview..."); render_view(c3, "/tmp/fs_nw.png", "Factory 3-Floor Stairs — NW Overview")

# Close-up stair 1 — look from inside the open end (low-x side)
c4 = mujoco.MjvCamera()
c4.lookat[:] = [24,8,1.5]; c4.distance = 8; c4.azimuth = -15; c4.elevation = -20
print("Stair 1 close-up..."); render_view(c4, "/tmp/fs_stair1.png", "Stairwell 1→2 Close-up (18cm steps)")

renderer.close()
print("All done!")
'''

# === Deploy ===
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
password = os.environ.get("S100P_PASSWORD")
if not password:
    raise SystemExit("S100P_PASSWORD is required")
ssh.connect("192.168.66.190", username="sunrise", password=password)


def run(cmd, t=120):
    _, o, e = ssh.exec_command(cmd, timeout=t)
    return o.read().decode('utf-8', 'replace').strip(), e.read().decode('utf-8', 'replace').strip()


def upload(path, content):
    sftp = ssh.open_sftp()
    with sftp.file(path, 'w') as f:
        f.write(content)
    sftp.close()


# Kill old
print("=== Kill old ===")
run('pkill -9 -f factory_ 2>/dev/null; pkill -9 -f mujoco 2>/dev/null; sleep 2')

# Upload scripts
print("=== Upload ===")
upload('/tmp/factory_stairs_viz.py', LIVE_VIZ)
upload('/tmp/render_stairs.py', RENDER_SCRIPT)
print(f"  Live viz: {len(LIVE_VIZ)} chars")
print(f"  Render: {len(RENDER_SCRIPT)} chars")

# Offscreen render first (quick)
print("\n=== Offscreen render ===")
out, err = run(
    'DISPLAY=:0 XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.QRJWL3 '
    'python3 /tmp/render_stairs.py 2>&1', t=60)
print(out)
if 'Traceback' in (out + (err or '')):
    print("RENDER ERR:", err[:500])

# Download renders
out2, _ = run('ls -la /tmp/fs_*.png 2>&1')
print(out2)

sftp = ssh.open_sftp()
for f in ['fs_se.png', 'fs_east.png', 'fs_nw.png', 'fs_stair1.png']:
    try:
        sftp.get(f'/tmp/{f}', f'C:/tmp/{f}')
        sz = os.path.getsize(f'C:/tmp/{f}')
        print(f'OK {f} ({sz // 1024} KB)')
    except Exception as ex:
        print(f'FAIL {f}: {ex}')
sftp.close()

# Launch live viz
print("\n=== Launch live viz ===")
launch_cmd = (
    "nohup bash -c '"
    "export DISPLAY=:0; "
    "export XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.QRJWL3; "
    "export XDG_RUNTIME_DIR=/run/user/1000; "
    "export PYTHONUNBUFFERED=1; "
    "python3 /tmp/factory_stairs_viz.py"
    "' > /tmp/factory_stairs.log 2>&1 &"
)
run(launch_cmd)
print("Launched! Monitoring...")

for i in range(25):
    time.sleep(8)
    out, _ = run('tail -12 /tmp/factory_stairs.log 2>/dev/null')
    print(f"\n--- {8*(i+1)}s ---")
    print(out)
    if 'GOAL REACHED' in (out or ''):
        break
    if 'Traceback' in (out or ''):
        out2, _ = run('tail -30 /tmp/factory_stairs.log 2>/dev/null')
        print(out2)
        break

print("\n=== Final ===")
out, _ = run('tail -15 /tmp/factory_stairs.log 2>/dev/null')
print(out)
out, _ = run('DISPLAY=:0 wmctrl -l 2>/dev/null')
print("Window:", out)

ssh.close()
