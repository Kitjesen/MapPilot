import { lazy, Suspense, useState } from 'react'
import { Activity, Box, Camera } from 'lucide-react'
import styles from './SensorResult.module.css'
import { useImuMonitor } from '../../hooks/useImuMonitor.ts'
import { useSensorMonitor } from '../../hooks/useSensorMonitor.ts'
import { sensorCards } from '../../services/sensors.ts'
import { LiveCameraCard } from './LiveCameraCard'

const Scene = lazy(() => import('../MapGalleryScene.tsx').then(module => ({ default: module.MapGalleryScene })))
const labels: Record<string, string> = { imu: 'IMU', joints: '关节', motors: '电机', voltages: '电压', lidar: 'LiDAR 扫描', camera: '相机', navigation: '导航诊断' }
const number = (value: unknown) => typeof value === 'number' && Number.isFinite(value) ? value.toFixed(3) : '—'
const object = (value: unknown): Record<string, unknown> => value && typeof value === 'object' && !Array.isArray(value) ? value as Record<string, unknown> : {}

export function SensorResult({ result, monitorLink = true }: { result: Record<string, unknown>; monitorLink?: boolean }) {
  const [open, setOpen] = useState(false)
  const monitor = useImuMonitor()
  const sensors = useSensorMonitor()
  const sensor = String(result.sensor ?? '')
  const values = object(result.values)
  const rows = Array.isArray(values.joints) ? values.joints.map(object) : Array.isArray(values.motors) ? values.motors.map(object) : []
  const points = Array.isArray(result.points) && result.points.every(value => typeof value === 'number' && Number.isFinite(value)) ? result.points as number[] : []
  const image = typeof result.image_data_url === 'string' && result.image_data_url.startsWith('data:image/jpeg;base64,') ? result.image_data_url : ''
  const elapsed = object(values.source_elapsed)
  return <section className={styles.card} aria-label={sensor ? `${labels[sensor] ?? sensor}观测` : '机器人能力'}>
    <header><Activity size={14} /><strong>{labels[sensor] ?? '机器人能力'}</strong><span>{String(result.robot ?? result.provider ?? '只读观测')}</span></header>
    {result.sensors != null && <div className={styles.capabilities}>{Object.entries(object(result.sensors)).map(([name, raw]) => {
      const item = object(raw)
      return <span key={name} data-configured={item.configured === true}>{labels[name] ?? name} · {item.configured ? '已配置' : '未配置'}{typeof item.joint_count === 'number' ? ` · ${item.joint_count} 关节` : ''}</span>
    })}<p>已配置表示存在数据适配器，是否在线以实际读取为准。</p></div>}
    {monitorLink && sensor === 'imu' && monitor && <button type="button" onClick={monitor}>打开实时监测与记录</button>}
    {monitorLink && sensor !== 'imu' && sensors && sensorCards.filter(item => item.id === sensor).map(item => <button type="button" key={item.id} onClick={() => sensors(item.id)}>打开{item.name}监测</button>)}
    {sensor === 'imu' && <dl className={styles.vectors}>{[
      ['角速度 rad/s', values.angular_velocity_rad_s, 'xyz'],
      ['线加速度 m/s²', values.linear_acceleration_m_s2, 'xyz'],
      ['姿态四元数', values.orientation_world_to_body, 'wxyz'],
    ].map(([label, raw, axes]) => <div key={String(label)}><dt>{String(label)}</dt><dd>{String(axes).split('').map(axis => <span key={axis}>{axis} <b>{number(object(raw)[axis])}</b></span>)}</dd></div>)}</dl>}
    {rows.length > 0 && <div className={styles.scroll}><table><thead><tr>{sensor === 'joints'
      ? <><th>关节</th><th>位置 rad</th><th>速度 rad/s</th><th>力矩 N·m</th></>
      : <><th>电机</th><th>在线</th><th>温度 °C</th><th>状态 / 错误</th></>}</tr></thead><tbody>{rows.map((row, index) => <tr key={index}>
        <th>{String(row.name ?? row.id ?? index)}</th>{sensor === 'joints'
          ? <><td>{number(row.position_rad)}</td><td>{number(row.velocity_rad_s)}</td><td>{number(row.torque_nm)}</td></>
          : <><td>{typeof row.online === 'boolean' ? row.online ? '是' : '否' : '—'}</td><td>{number(row.temperature_c)}</td><td>{String(row.status_code ?? '—')} / {Array.isArray(row.errors) ? row.errors.join(', ') || '无' : '—'}</td></>}
      </tr>)}</tbody></table></div>}
    {Array.isArray(values.volts) && <div className={styles.volts}>{values.volts.map((value, index) => <span key={index}>{index} · {number(value)} V</span>)}</div>}
    {image && <img className={styles.image} src={image} alt="机器人相机快照" />}
    {sensor === 'camera' && <p><Camera size={12} />相机快照可能来自缓存；此接口未报告拍摄时间。</p>}
    {sensor === 'camera' && <LiveCameraCard />}
    {sensor === 'lidar' && <><p><Box size={12} />{String(result.point_count ?? '—')} 点 · {String(result.frame_id || '坐标系未报告')}{result.preview_sample === true ? ' · 抽样显示' : ''}</p>
      <button type="button" onClick={() => setOpen(!open)}>{open ? '收起扫描' : '查看三维扫描'}</button>
      {open && (points.length ? <div className={styles.viewer}><Suspense fallback={<p role="status">正在加载扫描…</p>}>
        <Scene name="机器人 LiDAR 扫描" points={points} voxelScale={1} pointSize={2} opacity={1} ceiling={null} grid={false} gridStep={1} />
      </Suspense></div> : <p>这帧扫描没有可显示的点。</p>)}<p>显示点云用于查看；碰撞与制动判断由原生导航提供。</p></>}
    {sensor === 'navigation' && <p>已读取原生导航诊断。展开执行详情查看完整状态、来源时间与停车原因。</p>}
    {result.source_stamp_s != null && <p>来源时间 {String(result.source_stamp_s)}{typeof result.source_clock === 'string' ? ` · ${result.source_clock}` : ' · 来源时钟未标明'}</p>}
    {elapsed.seconds != null && <p>来源运行时间 {String(elapsed.seconds)} s · {String(elapsed.nanos ?? 0)} ns</p>}
    {result.fresh === false && <p role="status" className={styles.stale}>这份观测已经过期，不能作为当前状态。</p>}
  </section>
}
