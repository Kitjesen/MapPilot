import { SurfaceSettings } from './SurfaceSettings.tsx'
import { lazy, Suspense, useEffect, useId, useRef, useState } from 'react'
import { createPortal } from 'react-dom'
import { Activity, ArrowLeft, Check, CheckCircle2, ChevronRight, CircleAlert, Download, LocateFixed, Map, Network, Palette, RefreshCw, SlidersHorizontal, Stethoscope, X } from 'lucide-react'
import { useMapSettings } from '../hooks/useMapSettings.ts'
import { NoGoSettings } from './NoGoSettings.tsx'
import type { HealthResponse, SSEState, ToastKind } from '../types'
import * as api from '../services/api'
import { resetAllLayouts } from './floatingWidgetLayout'
import type { ResolvedTheme, Theme } from './useTheme'
import { text, type Locale } from '../i18n'
import styles from './SettingsMenu.module.css'

const SlamStatusPanel = lazy(() => import('./SlamStatusPanel').then(module => ({ default: module.SlamStatusPanel })))
const RuntimeDataflowView = lazy(() => import('./RuntimeDataflowView').then(module => ({ default: module.RuntimeDataflowView })))
const PlannerTuning = lazy(() => import('./PlannerTuning').then(module => ({ default: module.PlannerTuning })))
const LocalizationCard = lazy(() => import('./LocalizationCard').then(module => ({ default: module.LocalizationCard })))
const SensorSettings = lazy(() => import('./SensorSettings').then(module => ({ default: module.SensorSettings })))

interface SettingsMenuProps {
  open: boolean
  onClose: () => void
  theme: Theme
  resolvedTheme: ResolvedTheme
  onThemeChange: (theme: Theme) => void
  locale: Locale
  onLocaleChange: (locale: Locale) => void
  sseState: SSEState
  localNowS: number
  showToast: (message: string, kind?: ToastKind) => void
}

type Section = 'appearance' | 'map' | 'sensors' | 'diagnostics' | 'system'
type Diagnostic = 'slam' | 'dataflow' | 'planner'

const DIAGNOSTICS = [
  { value: 'slam' as const, icon: LocateFixed, en: 'Localization diagnostics', zh: '定位诊断',
    detailEn: 'SLAM status and localization quality.', detailZh: '查看 SLAM 状态与定位质量。' },
  { value: 'dataflow' as const, icon: Network, en: 'Data diagnostics', zh: '数据诊断',
    detailEn: 'Runtime dataflow and topic samples.', detailZh: '检查运行时数据链路与话题采样。' },
  { value: 'planner' as const, icon: SlidersHorizontal, en: 'Planning parameters', zh: '规划参数',
    detailEn: 'Inspect parameters and validate map plans.', detailZh: '检查参数并验证地图规划。' },
]

const THEMES: { value: Theme; en: string; zh: string }[] = [
  { value: 'system', en: 'System', zh: '跟随系统' },
  { value: 'light', en: 'Light', zh: '浅色' },
  { value: 'dark', en: 'Dark', zh: '深色' },
]

function ThemePreview({ theme }: { theme: Theme }) {
  const clipId = useId()
  const canvas = (dark: boolean) => <g>
    <rect width="180" height="112" rx="9" fill={dark ? '#171717' : '#fafafa'} />
    <path d="M0 25H180M48 25V112" stroke={dark ? '#383838' : '#dedede'} />
    <circle cx="12" cy="13" r="2.5" fill={dark ? '#777' : '#bbb'} />
    <circle cx="21" cy="13" r="2.5" fill={dark ? '#555' : '#ddd'} />
    <rect x="9" y="37" width="29" height="6" rx="3" fill={dark ? '#6b6b6b' : '#c1c1c1'} />
    <rect x="9" y="53" width="23" height="5" rx="2.5" fill={dark ? '#3f3f3f' : '#e2e2e2'} />
    <rect x="9" y="68" width="26" height="5" rx="2.5" fill={dark ? '#3f3f3f' : '#e2e2e2'} />
    <rect x="61" y="38" width="76" height="7" rx="3.5" fill={dark ? '#dedede' : '#414141'} />
    <rect x="61" y="54" width="98" height="5" rx="2.5" fill={dark ? '#525252' : '#d4d4d4'} />
    <rect x="61" y="66" width="82" height="5" rx="2.5" fill={dark ? '#525252' : '#d4d4d4'} />
    <rect x="61" y="85" width="31" height="13" rx="4" fill={dark ? '#ededed' : '#303030'} />
  </g>
  return <svg className={styles.themePreview} viewBox="0 0 180 112" aria-hidden="true">
    {canvas(theme === 'dark')}
    {theme === 'system' && <>
      <defs><clipPath id={clipId}><rect x="90" width="90" height="112" /></clipPath></defs>
      <g clipPath={`url(#${clipId})`}>{canvas(true)}</g>
    </>}
  </svg>
}

function SystemSettings({ locale }: { locale: Locale }) {
  const [snapshot, setSnapshot] = useState<{ data: HealthResponse; receivedAt: Date } | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [loading, setLoading] = useState(true)
  const [revision, setRevision] = useState(0)

  useEffect(() => {
    let active = true
    api.fetchHealth().then(data => {
      if (active) setSnapshot({ data, receivedAt: new Date() })
    }).catch((reason: unknown) => {
      if (active) setError(reason instanceof Error ? reason.message : String(reason))
    }).finally(() => {
      if (active) setLoading(false)
    })
    return () => { active = false }
  }, [revision])

  const modules = Object.entries(snapshot?.data.modules ?? {})
  const okCount = modules.filter(([, status]) => status === 'ok').length
  return <>
    <div className={styles.sectionHeading}>
      <div><h3>{text(locale, 'System health', '系统健康')}</h3>
        <p>{text(locale, 'View the current status of system components.', '查看当前功能模块的状态。')}</p></div>
      <button className={styles.iconButton} disabled={loading}
        aria-label={text(locale, 'Refresh health', '刷新健康状态')} onClick={() => {
          setError(null); setLoading(true); setRevision(value => value + 1)
        }}><RefreshCw size={16} className={loading ? styles.spin : undefined} /></button>
    </div>
    {error && <p className={styles.error} role="alert">{text(locale, 'Health check failed', '健康状态读取失败')}：{error}</p>}
    {snapshot ? <>
      <div className={styles.healthSummary}>
        <span><strong>{okCount}</strong>{text(locale, ' healthy', ' 正常')}</span>
        <span><strong>{modules.length - okCount}</strong>{text(locale, ' issues', ' 异常')}</span>
        <span className={styles.timestamp}>{text(locale, 'Updated ', '更新于 ')}
          {snapshot.receivedAt.toLocaleTimeString(locale === 'zh' ? 'zh-CN' : 'en-US', { hour12: false })}</span>
      </div>
      {modules.length > 0 ? <ul className={styles.moduleList} aria-label={text(locale, 'Module status', '模块状态')}>
        {modules.map(([name, status]) => <li key={name}>
          <span className={styles.moduleName}>{name}</span>
          <span className={status === 'ok' ? styles.moduleOk : styles.moduleIssue}>
            {status === 'ok' ? <CheckCircle2 size={14} /> : <CircleAlert size={14} />}
            {status === 'ok' ? text(locale, 'Healthy', '正常') : status}
          </span>
        </li>)}
      </ul> : <p className={styles.help}>{text(locale, 'The server returned no module status.', '服务端尚未返回模块状态。')}</p>}
    </> : loading && <p className={styles.help} role="status">{text(locale, 'Reading system health…', '正在读取系统健康状态…')}</p>}
    <div className={styles.downloadRow}>
      <div><h3>{text(locale, 'Diagnostic report', '诊断资料')}</h3>
        <p>{text(locale, 'Download logs, configuration and the current health snapshot.', '下载日志、配置与当前健康快照。')}</p></div>
      <a className={styles.actionButton} href="/api/v1/diagnostic_pack" download>
        <Download size={15} />{text(locale, 'Download', '下载诊断包')}
      </a>
    </div>
  </>
}

export function SettingsMenu({ open, onClose, theme, resolvedTheme, onThemeChange, locale, onLocaleChange, sseState, localNowS, showToast }: SettingsMenuProps) {
  const panel = useRef<HTMLElement>(null)
  const mapSettings = useMapSettings()
  const [selection, setSelection] = useState<{ section: Section; request: number }>({ section: 'appearance', request: 0 })
  const [diagnostic, setDiagnostic] = useState<Diagnostic | null>(null)
  const section = (mapSettings?.request ?? 0) > selection.request ? 'map' : selection.section
  useEffect(() => {
    if (!open) return
    const previousFocus = document.activeElement as HTMLElement | null
    panel.current?.querySelector<HTMLButtonElement>('button')?.focus()
    const onKey = (event: KeyboardEvent) => {
      if (event.key === 'Escape') { event.preventDefault(); onClose() }
      if (event.key !== 'Tab') return
      const controls = [...(panel.current?.querySelectorAll<HTMLElement>('button:not(:disabled), a[href], input:not(:disabled), select:not(:disabled), summary') ?? [])]
        .filter(control => control.getClientRects().length > 0
          && !control.matches('details:not([open]) > :not(summary), details:not([open]) > :not(summary) *'))
      if (!controls.length) return
      const firstControl = controls[0], lastControl = controls[controls.length - 1]
      if (event.shiftKey && document.activeElement === firstControl) {
        event.preventDefault(); lastControl.focus()
      } else if (!event.shiftKey && document.activeElement === lastControl) {
        event.preventDefault(); firstControl.focus()
      }
    }
    document.addEventListener('keydown', onKey)
    return () => { document.removeEventListener('keydown', onKey); previousFocus?.focus() }
  }, [open, onClose])
  useEffect(() => {
    if (open && section === 'diagnostics') panel.current?.querySelector<HTMLButtonElement>('[data-diagnostic-focus]')?.focus()
  }, [open, section, diagnostic])
  useEffect(() => {
    if (!open || section !== 'map' || mapSettings?.focus !== 'no-go') return
    const target = panel.current?.querySelector<HTMLDetailsElement>('[data-map-section="no-go"]')
    if (!target) return
    target.open = true
    target.scrollIntoView({ block: 'start' })
    target.querySelector('summary')?.focus()
  }, [open, section, mapSettings?.focus, mapSettings?.request, mapSettings?.map?.name, mapSettings?.map?.contentEpoch])
  if (!open) return null
  const diagnosticEntry = DIAGNOSTICS.find(item => item.value === diagnostic)
  const categories = [
    { value: 'appearance' as const, icon: Palette, en: 'Appearance', zh: '外观' },
    { value: 'map' as const, icon: Map, en: 'Map', zh: '地图' },
    { value: 'sensors' as const, icon: Activity, en: 'Sensors', zh: '传感器' },
    { value: 'diagnostics' as const, icon: Stethoscope, en: 'Diagnostics', zh: '诊断' },
    { value: 'system' as const, icon: Activity, en: 'System', zh: '系统' },
  ]
  return createPortal(<div className={styles.backdrop} onClick={onClose}>
    <aside ref={panel} className={`${styles.panel} ${section === 'diagnostics' && diagnostic ? styles.expandedPanel : ''}`} role="dialog" aria-modal="true"
      aria-label={text(locale, 'Settings', '设置')} onClick={event => event.stopPropagation()}>
      <header className={styles.header}><h2>{text(locale, 'Settings', '设置')}</h2>
        <nav className={styles.categories} aria-label={text(locale, 'Settings categories', '设置分类')}>
          {categories.map(({ value, icon: Icon, en, zh }) => <button key={value}
            aria-current={section === value ? 'page' : undefined}
            onClick={() => setSelection({ section: value, request: mapSettings?.request ?? 0 })}><Icon size={17} />{text(locale, en, zh)}</button>)}
        </nav>
        <button className={styles.iconButton} aria-label={text(locale, 'Close settings', '关闭设置')} onClick={onClose}><X size={19} /></button>
      </header>
      <div className={styles.body}>
        <div className={`${styles.content} ${section === 'diagnostics' && diagnostic ? styles.diagnosticContent : ''}`}>
          {section === 'appearance' && <>
            <div className={styles.sectionHeading}><h3>{text(locale, 'Theme', '界面主题')}</h3></div>
            <div className={styles.themeOptions}>{THEMES.map(option => <button key={option.value}
              className={styles.themeOption} aria-pressed={theme === option.value}
              onClick={() => onThemeChange(option.value)}>
              <ThemePreview theme={option.value} />
              <span className={styles.themeLabel}>{text(locale, option.en, option.zh)}
                <span className={styles.selectedMark} aria-hidden="true"><Check size={12} strokeWidth={2.5} /></span>
              </span>
            </button>)}</div>
            {theme === 'system' && <p className={styles.themeNote}>
              {text(locale, 'Currently following the ', '当前跟随系统使用')}
              {resolvedTheme === 'dark' ? text(locale, 'dark appearance.', '深色外观。') : text(locale, 'light appearance.', '浅色外观。')}
            </p>}
            <section className={styles.languageSection}>
              <div><h3>{text(locale, 'Language', '显示语言')}</h3><p>{text(locale, 'Applied immediately.', '选择后立即生效。')}</p></div>
              <div className={styles.languageOptions}>
                <button aria-pressed={locale === 'zh'} onClick={() => onLocaleChange('zh')}>中文</button>
                <button aria-pressed={locale === 'en'} onClick={() => onLocaleChange('en')}>English</button>
              </div>
            </section>
            <div className={styles.resetRow}><div><h3>{text(locale, 'Panel layout', '面板布局')}</h3>
              <p>{text(locale, 'Restore panel positions and refresh this page.', '恢复面板位置并刷新页面。')}</p></div>
              <button className={styles.actionButton} onClick={resetAllLayouts}><RefreshCw size={14} />{text(locale, 'Reset layout', '重置布局')}</button>
            </div>
          </>}
          {section === 'system' && <SystemSettings locale={locale} />}
          {section === 'sensors' && <Suspense fallback={<p role="status">正在加载传感器设置…</p>}><SensorSettings locale={locale} onOpen={onClose} /></Suspense>}
          {section === 'diagnostics' && (diagnosticEntry ? <>
            <div className={styles.diagnosticToolbar}>
              <button className={styles.actionButton} data-diagnostic-focus onClick={() => setDiagnostic(null)}>
                <ArrowLeft size={15} />{text(locale, 'Back to diagnostics', '返回诊断列表')}
              </button>
              <h3>{text(locale, diagnosticEntry.en, diagnosticEntry.zh)}</h3>
            </div>
            <div className={styles.diagnosticView}>
              <Suspense fallback={<p role="status">{text(locale, 'Loading diagnostics…', '正在加载诊断…')}</p>}>
                {diagnostic === 'slam' && <div className={styles.localizationDiagnostics}>
                  <SlamStatusPanel sseState={sseState} showToast={showToast} locale={locale} />
                  <details>
                    <summary>{text(locale, 'Satellite and fusion', '卫星与融合')}</summary>
                    <LocalizationCard sseState={sseState} localNowS={localNowS} locale={locale} />
                  </details>
                </div>}
                {diagnostic === 'dataflow' && <RuntimeDataflowView sseState={sseState} />}
                {diagnostic === 'planner' && <PlannerTuning showToast={showToast} />}
              </Suspense>
            </div>
          </> : <>
            <div className={styles.sectionHeading}><div><h3>{text(locale, 'Diagnostics', '诊断')}</h3>
              <p>{text(locale, 'Inspect localization, dataflow and planning.', '查看定位状态、数据链路与规划参数。')}</p></div></div>
            <div className={styles.diagnosticList}>
              {DIAGNOSTICS.map(({ value, icon: Icon, en, zh, detailEn, detailZh }, index) => <button key={value}
                data-diagnostic-focus={index === 0 ? '' : undefined} onClick={() => setDiagnostic(value)}>
                <Icon size={19} /><span><strong>{text(locale, en, zh)}</strong><small>{text(locale, detailEn, detailZh)}</small></span><ChevronRight size={16} />
              </button>)}
            </div>
          </>)}
          {section === 'map' && <>
            <div className={styles.sectionHeading}><div><h3>点云清洗</h3><p>{mapSettings?.editing
              ? '参数应用到当前编辑草稿，预览后另存地图。' : '先在已保存地图中点击“清洗”。'}</p></div></div>
            <div ref={mapSettings?.setTarget} />
            {mapSettings && <SurfaceSettings />}
            {mapSettings?.map && <NoGoSettings key={`${mapSettings.map.name}:${mapSettings.map.contentEpoch}`}
              map={mapSettings.map} editing={mapSettings.editing} />}
          </>}
        </div>
      </div>
    </aside>
  </div>, document.body)
}
