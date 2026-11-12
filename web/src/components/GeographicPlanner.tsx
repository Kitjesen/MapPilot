import { useEffect, useRef, useState } from 'react'
import { ArrowDown, ArrowUp, Download, Globe2, LocateFixed, MapPin, Plus, Trash2, Upload, X } from 'lucide-react'
import { ModalShell } from './Modal'
import { InspectionCalendar } from './InspectionCalendar'
import * as api from '../services/api'
import type { InspectionRoute, InspectionTaskStatusResponse, MapInfo } from '../types'
import { text, type Locale } from '../i18n'
import { dashboardFetch } from '../services/observationMode'
import { fetchAMapConfig, amapPositions, loadAMap, type AMapAPI } from '../services/amap'
import { GEOGRAPHIC_DRAFT_KEY, MAX_GEOGRAPHIC_POINTS, geographicPoint, parseGeographicPoints,
  serializeGeographicPoints, type GeographicDatum, type GeographicPoint } from '../services/geographicPoints'
import styles from './GeographicPlanner.module.css'

function InspectionPlans({ locale }: { locale: Locale }) {
  const [maps, setMaps] = useState<MapInfo[]>([])
  const [mapId, setMapId] = useState('')
  const [routes, setRoutes] = useState<InspectionRoute[]>([])
  const [error, setError] = useState('')
  const [taskId, setTaskId] = useState('')
  const [task, setTask] = useState<InspectionTaskStatusResponse | null>(null)
  useEffect(() => {
    let active = true
    void api.fetchMaps().then(result => {
      if (active) { setMaps(result); setMapId(result.find(map => map.is_active)?.name ?? result[0]?.name ?? '') }
    })
      .catch(reason => { if (active) setError(String(reason.message)) })
    return () => { active = false }
  }, [])
  useEffect(() => {
    if (!mapId) return
    let active = true
    void api.fetchInspectionRoutes(mapId).then(result => { if (active) setRoutes(result.routes) })
      .catch(reason => { if (active) setError(reason.message === 'inspection service is unavailable' ? '巡检路线服务未连接' : String(reason.message)) })
    return () => { active = false }
  }, [mapId])
  useEffect(() => {
    if (!taskId) return
    let active = true
    void api.fetchInspectionTask(taskId).then(result => { if (active) setTask(result) })
      .catch(reason => { if (active) setError(String(reason.message)) })
    return () => { active = false }
  }, [taskId])
  return <>
    <InspectionCalendar key={mapId} locale={locale} routes={routes} routeUnavailable={Boolean(error)}
      routeSettings={<details className={styles.routeSource}><summary>路线来源</summary>
        <label className={styles.mapChoice}><span>地图</span><select aria-label="巡检计划本地地图" value={mapId} onChange={event => {
          setMapId(event.target.value); setRoutes([]); setError(''); setTaskId(''); setTask(null)
        }}>{!maps.length && <option value="">暂无已保存地图</option>}{maps.map(map => <option key={map.name} value={map.name}>{map.name}</option>)}</select></label>
        {error && <p className={styles.error} role="status">{error}</p>}
      </details>} onOpenTask={id => { setTask(null); setError(''); setTaskId(id) }} />
    {taskId && <section className={styles.taskSnapshot} aria-label="计划任务回执"><strong>任务回执</strong>
      <span>{taskId}</span><span>{task ? task.current_state : error ? `回执读取失败：${error}` : '正在读取…'}</span>
      {task && <span>{task.execution_confirmed ? '执行状态已确认' : '尚未确认实际执行'}{task.reason ? ` · ${task.reason}` : ''}</span>}
      <button onClick={() => { setTaskId(''); setTask(null) }}>收起</button></section>}
  </>
}

function CityMap({ points, onPick }: { points: GeographicPoint[]; onPick: (longitude: number, latitude: number) => void }) {
  const container = useRef<HTMLDivElement>(null)
  const pick = useRef(onPick)
  const [instance, setInstance] = useState<{ api: AMapAPI; map: InstanceType<AMapAPI['Map']> } | null>(null)
  const [error, setError] = useState('')
  useEffect(() => { pick.current = onPick }, [onPick])
  const [mapState, setMapState] = useState('loading')
  useEffect(() => {
    let stopped = false
    const controller = new AbortController()
    const timeout = window.setTimeout(() => controller.abort(), 6000)
    let map: InstanceType<AMapAPI['Map']> | undefined
    void fetchAMapConfig(controller.signal).then(async config => {
      window.clearTimeout(timeout)
      if (stopped) return
      setMapState(config.state)
      if (config.state !== 'configured') return
      const api = await loadAMap(config)
      if (stopped || !container.current) return
      map = new api.Map(container.current, { zoom: 17, viewMode: '2D' })
      map.on('click', event => pick.current(event.lnglat.getLng(), event.lnglat.getLat()))
      setInstance({ api, map })
    }).catch(reason => { if (!stopped) { setMapState('failed'); setError(controller.signal.aborted ? '地图配置读取超时' : String(reason.message)) } })
    return () => { stopped = true; window.clearTimeout(timeout); controller.abort(); map?.destroy() }
  }, [])
  useEffect(() => {
    if (!instance) return
    let stopped = false
    instance.map.clearMap()
    void amapPositions(instance.api, points).then(positions => {
      if (stopped) return
      setError('')
      instance.map.add(positions.map((position, index) => new instance.api.Marker({ position,
        title: points[index].name, label: { content: String(index + 1), direction: 'top' } })))
      if (positions.length) instance.map.setFitView()
    }).catch(reason => { if (!stopped) setError(String(reason.message)) })
    return () => { stopped = true }
  }, [instance, points])
  return <div className={styles.mapPanel}>
    <div ref={container} className={styles.mapCanvas} />
    {['missing', 'incomplete', 'invalid'].includes(mapState) && <div className={styles.mapPlaceholder}>
      <Globe2 size={36} strokeWidth={1.2} /><strong>{mapState === 'invalid' ? '地图配置格式有误' : '城市底图未配置'}</strong><span>可先导入 GPS 点位或输入经纬度</span>
    </div>}
    {['loading', 'configured'].includes(mapState) && !instance && !error && <span className={styles.mapMessage}>正在加载高德地图…</span>}
    {error && <span className={styles.mapMessage} role="status">{error}</span>}
    {instance && !error && <span className={styles.mapMessage}>点击地图选点 · GCJ-02</span>}
  </div>
}

function GeographicPlanner({ locale, onClose }: { locale: Locale; onClose: () => void }) {
  const [tab, setTab] = useState<'points' | 'plans'>('points')
  const [initial] = useState(() => {
    try {
      const stored = localStorage.getItem(GEOGRAPHIC_DRAFT_KEY)
      return { points: stored ? parseGeographicPoints(stored) : [], error: '' }
    } catch { return { points: [], error: '本机草稿读取失败；原记录未覆盖，请先检查浏览器存储' } }
  })
  const [points, setPoints] = useState<GeographicPoint[]>(initial.points)
  const [error, setError] = useState(initial.error)
  const [notice, setNotice] = useState('')
  const [name, setName] = useState('')
  const [longitude, setLongitude] = useState('')
  const [latitude, setLatitude] = useState('')
  const [datum, setDatum] = useState<GeographicDatum>('WGS84')
  const [locating, setLocating] = useState(false)
  const fileInput = useRef<HTMLInputElement>(null)
  const locationRequest = useRef<AbortController | null>(null)
  useEffect(() => () => locationRequest.current?.abort(), [])

  function update(next: GeographicPoint[]) {
    if (next.length > MAX_GEOGRAPHIC_POINTS) throw new Error('最多保存 500 个点位')
    // Save before replacing the visible draft so quota failures cannot look successful.
    localStorage.setItem(GEOGRAPHIC_DRAFT_KEY, serializeGeographicPoints(next))
    setPoints(next)
    setError('')
    setNotice('已保存到此浏览器')
  }
  function change(next: GeographicPoint[]) {
    try { update(next) } catch (reason) { setError(reason instanceof Error ? reason.message : '草稿保存失败') }
  }
  async function importFile(file: File) {
    try {
      if (file.size > 1_000_000) throw new Error('点位文件不能超过 1 MB')
      const imported = parseGeographicPoints(await file.text())
      if (!imported.length) throw new Error('文件中没有点位')
      update([...points, ...imported])
      setNotice(`已导入 ${imported.length} 个点位 · 仅保存草稿`)
    } catch (reason) { setError(reason instanceof Error ? reason.message : '导入失败') }
  }
  async function locate() {
    setLocating(true)
    setError('')
    const controller = new AbortController()
    locationRequest.current = controller
    const timeout = window.setTimeout(() => controller.abort(), 6000)
    try {
      const response = await dashboardFetch('/api/v1/geography/location', { signal: controller.signal, cache: 'no-store' })
      if (!response.ok) throw new Error('机器人定位接口不可用')
      const result = await response.json()
      if (result.state !== 'ready' || !result.location) {
        const reasons: Record<string, string> = { unavailable: '未接入机器人 GNSS', no_fix: '机器人 GNSS 尚未定位',
          stale: '机器人 GNSS 数据已过期', invalid: '机器人 GNSS 数据异常' }
        throw new Error(reasons[result.state] ?? '无法读取机器人定位')
      }
      const point = geographicPoint({ ...result.location, name: '机器人位置', datum: 'WGS84' })
      setLongitude(String(point.longitude)); setLatitude(String(point.latitude)); setDatum(point.datum)
      setName(point.name); setNotice('已填入机器人位置，添加后保存')
    } catch (reason) { if (!controller.signal.aborted) setError(reason instanceof Error ? reason.message : '定位读取失败')
      else setError('定位读取超时') }
    finally { window.clearTimeout(timeout); setLocating(false) }
  }
  function exportDraft() {
    const url = URL.createObjectURL(new Blob([serializeGeographicPoints(points)], { type: 'application/json' }))
    const anchor = document.createElement('a'); anchor.href = url; anchor.download = 'lingtu-geographic-points.json'; anchor.click()
    window.setTimeout(() => URL.revokeObjectURL(url), 1000)
  }
  return <ModalShell open title={text(locale, 'Campus map', '城市地图')} onClose={onClose} className={styles.dialog}
    headerAction={<button className={styles.close} data-modal-cancel onClick={onClose} aria-label="关闭城市地图"><X size={18} /></button>}>
    <div className={styles.tabs} role="tablist" aria-label="城市巡检视图">
      <button role="tab" aria-selected={tab === 'points'} onClick={() => setTab('points')}>地理点位</button>
      <button role="tab" aria-selected={tab === 'plans'} onClick={() => setTab('plans')}>巡检计划</button>
    </div>
    {tab === 'plans' ? <InspectionPlans locale={locale} /> : <>
    <div className={styles.toolbar}>
      <span>园区／街区 <b>{points.length} 个点位</b></span>
      <div><button onClick={() => fileInput.current?.click()}><Upload size={15} />导入点位</button>
        <button onClick={exportDraft} disabled={!points.length} aria-label="导出点位"><Download size={15} /></button></div>
      <input ref={fileInput} type="file" hidden accept=".json,.geojson,application/json,application/geo+json" onChange={event => {
        const file = event.target.files?.[0]; if (file) void importFile(file); event.target.value = ''
      }} />
    </div>
    <div className={styles.layout}>
      <CityMap points={points} onPick={(lng, lat) => { setLongitude(lng.toFixed(7)); setLatitude(lat.toFixed(7)); setDatum('GCJ02'); setNotice('地图选点已填入，添加后保存') }} />
      <section className={styles.points} aria-label="巡检地理点草稿">
        <header><strong>巡检点位</strong><span>草稿</span></header>
        <ol>{points.map((point, index) => <li key={index}>
          <span className={styles.number}>{index + 1}</span><div className={styles.pointText}><strong>{point.name}</strong>
            <span>{point.longitude.toFixed(6)}, {point.latitude.toFixed(6)}</span><small>{point.datum === 'GCJ02' ? 'GCJ-02' : 'WGS84'}</small></div>
          <div className={styles.pointActions}>
            <button aria-label={`上移 ${point.name}`} disabled={index === 0} onClick={() => { const next = [...points]; [next[index - 1], next[index]] = [next[index], next[index - 1]]; change(next) }}><ArrowUp size={13} /></button>
            <button aria-label={`下移 ${point.name}`} disabled={index === points.length - 1} onClick={() => { const next = [...points]; [next[index + 1], next[index]] = [next[index], next[index + 1]]; change(next) }}><ArrowDown size={13} /></button>
            <button aria-label={`删除 ${point.name}`} onClick={() => change(points.filter((_, i) => i !== index))}><Trash2 size={13} /></button>
          </div>
        </li>)}</ol>
        {!points.length && <div className={styles.empty}><MapPin size={22} /><span>添加第一个巡检点</span></div>}
      </section>
    </div>
    <form className={styles.entry} onSubmit={event => {
      event.preventDefault()
      try {
        if (!longitude.trim() || !latitude.trim()) throw new Error('请填写经度和纬度')
        update([...points, geographicPoint({ name: name || `点位 ${points.length + 1}`, longitude: Number(longitude), latitude: Number(latitude), datum })])
        setName(''); setLongitude(''); setLatitude('')
      } catch (reason) { setError(reason instanceof Error ? reason.message : '点位保存失败') }
    }}>
      <label><span>名称</span><input aria-label="点位名称" maxLength={80} placeholder="巡检点" value={name} onChange={e => setName(e.target.value)} /></label>
      <label><span>经度</span><input aria-label="经度" inputMode="decimal" required value={longitude} onChange={e => setLongitude(e.target.value)} /></label>
      <label><span>纬度</span><input aria-label="纬度" inputMode="decimal" required value={latitude} onChange={e => setLatitude(e.target.value)} /></label>
      <label><span>坐标系</span><select aria-label="坐标系" value={datum} onChange={e => setDatum(e.target.value as GeographicDatum)}><option value="WGS84">GPS · WGS84</option><option value="GCJ02">高德 · GCJ-02</option></select></label>
      <button type="submit" className={styles.add}><Plus size={15} />添加</button>
      <button type="button" disabled={locating} onClick={() => void locate()} aria-label="读取机器人 GPS 位置"><LocateFixed size={16} />{locating ? '读取中' : '机器人位置'}</button>
    </form>
    {error && <div className={styles.error} role="alert">{error}</div>}
    {!error && notice && <div className={styles.notice} role="status">{notice}</div>}
    <footer className={styles.footer}><span>尚未对齐本地地图</span><button disabled title="需完成地理坐标与当前本地地图标定，并通过机器人路径检查">生成可执行路线</button></footer>
    </>}
  </ModalShell>
}

export function GeographicPlannerButton({ locale, className }: { locale: Locale; className?: string }) {
  const [open, setOpen] = useState(false)
  return <><button type="button" className={className} onClick={() => setOpen(true)}><Globe2 size={15} />{text(locale, 'City map', '城市地图')}</button>
    {open && <GeographicPlanner locale={locale} onClose={() => setOpen(false)} />}</>
}
