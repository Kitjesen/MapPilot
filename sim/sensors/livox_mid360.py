"""
Livox MID-360 LiDAR — 双方案

方案 A（默认）：mujoco_ray_caster C++ plugin
  - 在 XML 里用 <sensor type="ray_caster_lidar"> 配置
  - 数据直接从 d.sensordata 读取，无需额外 Python 代码
  - 参考: https://github.com/Albusgive/mujoco_ray_caster

方案 B（fallback）：纯 Python mj_multiRay
  - 不需要编译 C++ plugin
  - 使用 OmniPerception 的 Livox 扫描模式参考
  - 参考: https://github.com/aCodeDog/OmniPerception

此文件实现方案 B，bridge.py 优先检测方案 A（plugin 数据），
fallback 到方案 B（Python 计算）。
"""
import numpy as np
import time
from typing import Optional


# ═══════════════════════════════════════════════════════════════════════════
# 方案 A：从 mujoco_ray_caster plugin 读取传感器数据
# ═══════════════════════════════════════════════════════════════════════════

def read_plugin_lidar(model, data, sensor_name: str = 'lidar_mid360') -> Optional[np.ndarray]:
    """
    从 mujoco_ray_caster plugin 读取点云数据（方案 A）。

    sensor_data_types 包含 "pos_w" 时，sensordata 存的是世界坐标 XYZ。
    数据格式：[x0,y0,z0, x1,y1,z1, ...] float64

    Returns:
        (N, 3) float32 or None if plugin not available
    """
    try:
        import mujoco
        sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
        if sensor_id < 0:
            return None

        # 获取 plugin state 中的 h_ray_num, v_ray_num
        plugin_id  = model.sensor_plugin[sensor_id]
        state_idx  = model.plugin_stateadr[plugin_id]
        h_rays = int(data.plugin_state[state_idx])
        v_rays = int(data.plugin_state[state_idx + 1])
        n_pts  = h_rays * v_rays

        # sensordata 偏移
        adr    = model.sensor_adr[sensor_id]
        raw    = data.sensordata[adr : adr + n_pts * 3]  # XYZ * n_pts

        pts = raw.reshape(-1, 3).astype(np.float32)

        # 过滤无效点（(0,0,0) 或 nan）
        valid = np.any(pts != 0, axis=1) & ~np.any(np.isnan(pts), axis=1)
        return pts[valid]

    except Exception:
        return None


# ═══════════════════════════════════════════════════════════════════════════
# 方案 B：纯 Python mj_multiRay（Livox MID-360 扫描模式）
# 参考 OmniPerception 的 Livox 模式实现
# ═══════════════════════════════════════════════════════════════════════════

class LivoxMid360Fallback:
    """
    纯 Python Livox MID-360 仿真（方案 B，无需编译 C++ plugin）。

    扫描模式参考 OmniPerception（CoRL 2025）的 Livox 非重复花瓣模式：
      - 水平: 360°（全向）
      - 垂直: -7° ~ +52°（下倾 7° + 上倾 52°）
      - 非重复: 黄金角螺旋采样，模拟 Livox 的花瓣轨迹

    使用:
        lidar = LivoxMid360Fallback(model, data)
        pts = lidar.scan()  # (N, 3) world frame
    """

    HFOV       = 2 * np.pi          # 360°
    VFOV_MIN   = np.deg2rad(-7.0)
    VFOV_MAX   = np.deg2rad(52.0)
    RANGE_MIN  = 0.10               # m
    RANGE_MAX  = 70.0               # m
    NOISE_STD  = 0.02               # m
    N_RAYS     = 6400               # 400×16，轻量；可调到 40000 for 真实密度
    GOLDEN_ANG = np.pi * (3 - np.sqrt(5))

    def __init__(self, model, data,
                 body_name: str = 'lidar_link',
                 add_noise: bool = True,
                 seed: int = 0):
        self.model     = model
        self.data      = data
        self.add_noise = add_noise
        self.rng       = np.random.default_rng(seed)
        self._frame    = 0

        import mujoco
        self._body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if self._body_id < 0:
            self._body_id = 0
            print(f'[LivoxFallback] body "{body_name}" not found, using world origin')

        self._dirs_local = self._build_pattern(self.N_RAYS)
        print(f'[LivoxFallback] MID-360 pattern: {self.N_RAYS} rays, '
              f'VFOV=[{np.degrees(self.VFOV_MIN):.0f}°,{np.degrees(self.VFOV_MAX):.0f}°]')

    def _build_pattern(self, n: int) -> np.ndarray:
        """Livox 非重复花瓣模式：黄金角螺旋 × VFOV 均匀分布。"""
        i   = np.arange(n, dtype=np.float64)
        ha  = (i * self.GOLDEN_ANG) % (2 * np.pi)
        va  = self.VFOV_MIN + i / n * (self.VFOV_MAX - self.VFOV_MIN)
        cv  = np.cos(va)
        return np.column_stack([cv * np.cos(ha), cv * np.sin(ha), np.sin(va)])

    def scan(self) -> np.ndarray:
        import mujoco
        pos  = self.data.xpos[self._body_id].copy()
        rmat = self.data.xmat[self._body_id].reshape(3, 3).copy()

        # 每帧旋转偏移（模拟非重复覆盖）
        ang = self._frame * 0.628
        self._frame += 1
        c, s = np.cos(ang), np.sin(ang)
        Rz = np.array([[c,-s,0],[s,c,0],[0,0,1]])
        dirs = self._dirs_local @ Rz.T @ rmat.T  # (N,3) world frame

        dist_out  = np.full(self.N_RAYS, -1.0)
        geomid_out = np.full(self.N_RAYS, -1, dtype=np.int32)

        mujoco.mj_multiRay(
            self.model, self.data,
            pos, dirs.flatten(),
            None, 1, self._body_id,
            dist_out, geomid_out, self.N_RAYS
        )

        valid = (dist_out >= self.RANGE_MIN) & (dist_out <= self.RANGE_MAX)
        if not valid.any():
            return np.zeros((0, 3), dtype=np.float32)

        pts = (pos + dirs[valid] * dist_out[valid, None]).astype(np.float32)
        if self.add_noise:
            pts += self.rng.normal(0, self.NOISE_STD, pts.shape).astype(np.float32)
        return pts


# ═══════════════════════════════════════════════════════════════════════════
# 统一接口：优先方案 A，fallback 方案 B
# ═══════════════════════════════════════════════════════════════════════════

class LidarSensor:
    """
    统一 LiDAR 接口：自动检测 mujoco_ray_caster plugin，
    若不可用则 fallback 到 Python mj_multiRay 实现。
    """

    def __init__(self, model, data, body_name: str = 'lidar_link',
                 sensor_name: str = 'lidar_mid360'):
        self.model       = model
        self.data        = data
        self.sensor_name = sensor_name
        self._fallback   = None

        # 检测 plugin 是否可用
        try:
            import mujoco
            sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
            if sid >= 0 and model.sensor_plugin[sid] >= 0:
                self._use_plugin = True
                print(f'[LidarSensor] Using mujoco_ray_caster plugin ({sensor_name})')
                return
        except Exception:
            pass

        self._use_plugin = False
        self._fallback = LivoxMid360Fallback(model, data, body_name=body_name)
        print(f'[LidarSensor] Plugin not found, using Python fallback (mj_multiRay)')

    def scan(self) -> np.ndarray:
        """返回点云，shape (N, 3) float32，世界坐标系。"""
        if self._use_plugin:
            pts = read_plugin_lidar(self.model, self.data, self.sensor_name)
            if pts is not None and len(pts) > 0:
                return pts
            # plugin 有数据但读取失败，fallback
        if self._fallback is not None:
            return self._fallback.scan()
        return np.zeros((0, 3), dtype=np.float32)
