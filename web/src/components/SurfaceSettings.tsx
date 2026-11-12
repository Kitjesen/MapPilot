import { useState } from 'react'
import { useMapSettings } from '../hooks/useMapSettings.ts'
import type { SurfaceOptions } from '../services/surfaceMap.ts'
import styles from './SettingsMenu.module.css'

const fields: Array<[keyof SurfaceOptions, string, string]> = [
  ['radius_m', '补面半径', 'm'], ['max_hole_area_m2', '最大洞面积', 'm²'],
  ['max_slope', '最大坡度', 'm/m'], ['headroom_m', '预览净空', 'm'],
]
export function SurfaceSettings() {
  const context = useMapSettings()
  const [draft, setDraft] = useState(context!.surfaceOptions)
  return <section className={styles.mapSection} aria-label="补面预览设置"><h3>补全支撑</h3>
    <p>仅用于三维补面预览，不修改原始点云或导航配置。</p>
    <details className={styles.surfaceNotes}><summary>预览说明</summary>
      <p>实测支撑为青色，推断为橙色。预览净空由此处指定，正式导航按所选机器人净空计算。</p>
    </details>
    <form className={styles.surfaceForm} onSubmit={event => { event.preventDefault(); context!.setSurfaceOptions(draft) }}>
      <div className={styles.surfaceFields}>{fields.map(([key, label, unit]) => <label key={key}
        title={key === 'max_slope' ? '拟合坡度为高差 / 水平距离，无量纲比例（m/m）' : undefined}>
        <span>{label}<small>{unit}</small></span>
        <input type="number" required step="0.01" min={key === 'headroom_m' ? .01 : 0} value={draft[key]}
          onChange={event => setDraft({ ...draft, [key]: event.currentTarget.valueAsNumber })} /></label>)}</div>
      <div className={styles.surfaceFooter}><button type="submit" className={styles.actionButton}>应用预览参数</button></div>
    </form>
  </section>
}
