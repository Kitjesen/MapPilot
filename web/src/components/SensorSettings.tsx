import { useEffect, useState } from 'react'
import { ChevronRight } from 'lucide-react'
import { SensorMark } from './sensors/SensorMark.tsx'
import { useSensorMonitor } from '../hooks/useSensorMonitor.ts'
import { readDisplay, sensorCards, sensorRequest, type DisplaySettings } from '../services/sensors.ts'
import { listMeasurements, type MeasurementStatus } from '../services/measurements.ts'
import { text, type Locale } from '../i18n'
import styles from './SensorMonitor.module.css'

export function SensorSettings({ locale, onOpen }: { locale: Locale; onOpen?: () => void }) {
  const open = useSensorMonitor()
  const [capabilities, setCapabilities] = useState<Record<string, unknown> | null>(null)
  const [measurements, setMeasurements] = useState<MeasurementStatus[] | null>(null)
  const [errors, setErrors] = useState<string[]>([])
  const [revision, setRevision] = useState(0)
  const [display, setDisplay] = useState(readDisplay)
  useEffect(() => {
    const abort = new AbortController()
    void Promise.allSettled([sensorRequest('/api/v1/robot/capabilities', abort.signal), listMeasurements(abort.signal)]).then(([robot, extra]) => {
      if (abort.signal.aborted) return
      setCapabilities(robot.status === 'fulfilled' ? robot.value : null)
      setMeasurements(extra.status === 'fulfilled' ? extra.value.sensors : null)
      setErrors([robot, extra].flatMap(result => result.status === 'rejected' ? [String(result.reason)] : []))
    })
    return () => abort.abort()
  }, [revision])
  function configure(next: DisplaySettings) {
    setDisplay(next)
    try { localStorage.setItem('lingtu.sensor-display', JSON.stringify(next)) }
    catch { setErrors(previous => [...previous, text(locale, 'Display settings could not be saved.', '显示设置未能保存到浏览器。')]) }
  }
  return <div className={styles.settings}>
    <div className={styles.intro}><div><h3>{text(locale, 'Sensors and cards', '传感器与卡片')}</h3><p>{text(locale, 'Open a monitor beside the conversation.', '在聊天旁打开监测，不打断对话。')}</p></div><button type="button" onClick={() => setRevision(value => value + 1)}>{text(locale, 'Refresh', '刷新来源')}</button></div>
    <div className={styles.catalog}>{sensorCards.map(item => {
      const source = (['imu','joints','motors','voltages'].includes(item.id)
        ? (capabilities?.sensors as Record<string, Record<string, unknown>> | undefined)?.[item.id]
        : capabilities?.[item.id]) as Record<string, unknown> | undefined
      const extra = ['thermal','acoustic','semantic'].includes(item.id)
      const declared = measurements?.filter(sensor => sensor.modality === item.id)
      let status = text(locale, 'Source unknown', '来源未读取')
      if (item.id === 'gnss') status = text(locale, 'Native position', '原生位置')
      else if (extra && measurements !== null) status = declared?.length
        ? `${declared.length} ${text(locale, 'declared', '个已声明')}` : text(locale, 'No observations yet', '暂无该类观测')
      else if (!extra && capabilities) status = source?.configured === true ? text(locale, 'Configured', '已配置')
        : item.id === 'camera' ? text(locale, 'Check media status', '打开查看图传状态') : text(locale, 'Not configured', '未配置')
      return <button type="button" key={item.id} disabled={!open} onClick={() => { open?.(item.id); onOpen?.() }}>
        <span className={styles.logo} style={{ color: item.color }}><SensorMark sensor={item.id} size={25} /></span>
        <span><strong>{locale === 'en' ? item.id === 'imu' ? 'IMU' : item.id === 'lidar' ? 'LiDAR' : item.id[0].toUpperCase() + item.id.slice(1) : item.name}</strong><small>{locale === 'en' ? item.en : item.detail}</small></span><em>{status}</em><ChevronRight size={14} />
      </button>
    })}</div>
    <p className={styles.note}>{text(locale, 'Configured sources may be offline. Each card shows actual readings or explains why they are unavailable.', '已配置不代表在线，以卡片收到的数据为准。未连接的传感器会说明原因。')}</p>
    <fieldset className={styles.preferences}><legend>{text(locale, 'Display defaults', '默认显示')}</legend>
      <label>{text(locale, 'Numeric refresh', '数值刷新')}<select value={display.interval} onChange={event => configure({ ...display, interval: Number(event.target.value) })}>{[500,1000,2000].map(value => <option value={value} key={value}>{value / 1000} s</option>)}</select></label>
      <label>{text(locale, 'Trace window', '曲线窗口')}<select value={display.window} onChange={event => configure({ ...display, window: Number(event.target.value) })}>{[15,60,300].map(value => <option value={value} key={value}>{value} s</option>)}</select></label>
      <label>{text(locale, 'LiDAR point size', '雷达点大小')}<select value={display.pointSize} onChange={event => configure({ ...display, pointSize: Number(event.target.value) })}>{[1,2,3].map(value => <option value={value} key={value}>{value} px</option>)}</select></label>
    </fieldset>
    <p className={styles.note}>{text(locale, 'Applied on the next card opening. These settings only affect this browser; sensor rates and calibration are unchanged.', '重新打开卡片时应用。只影响此浏览器的显示，不修改设备采样频率或标定。')}</p>
    {errors.map((error, index) => <p className={styles.error} role="status" key={index}>{error}</p>)}
  </div>
}
