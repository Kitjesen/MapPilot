import { useEffect, useId, useRef, useState } from 'react'
import { createPortal } from 'react-dom'
import { Activity, Check, CheckCircle2, CircleAlert, Download, Palette, RefreshCw, X } from 'lucide-react'
import type { HealthResponse } from '../types'
import * as api from '../services/api'
import { resetAllLayouts } from './floatingWidgetLayout'
import type { ResolvedTheme, Theme } from './useTheme'
import { text, type Locale } from '../i18n'
import styles from './SettingsMenu.module.css'

interface SettingsMenuProps {
  open: boolean
  onClose: () => void
  theme: Theme
  resolvedTheme: ResolvedTheme
  onThemeChange: (theme: Theme) => void
  locale: Locale
  onLocaleChange: (locale: Locale) => void
}

type Section = 'appearance' | 'system'

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

export function SettingsMenu({ open, onClose, theme, resolvedTheme, onThemeChange, locale, onLocaleChange }: SettingsMenuProps) {
  const panel = useRef<HTMLElement>(null)
  const [section, setSection] = useState<Section>('appearance')
  useEffect(() => {
    if (!open) return
    const previousFocus = document.activeElement as HTMLElement | null
    panel.current?.querySelector<HTMLButtonElement>('button')?.focus()
    const onKey = (event: KeyboardEvent) => {
      if (event.key === 'Escape') { event.preventDefault(); onClose() }
      if (event.key !== 'Tab') return
      const controls = panel.current?.querySelectorAll<HTMLElement>('button:not(:disabled), a[href]')
      if (!controls?.length) return
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
  if (!open) return null
  const categories = [
    { value: 'appearance' as const, icon: Palette, en: 'Appearance', zh: '外观' },
    { value: 'system' as const, icon: Activity, en: 'System', zh: '系统' },
  ]
  return createPortal(<div className={styles.backdrop} onClick={onClose}>
    <aside ref={panel} className={styles.panel} role="dialog" aria-modal="true"
      aria-label={text(locale, 'Settings', '设置')} onClick={event => event.stopPropagation()}>
      <header className={styles.header}><h2>{text(locale, 'Settings', '设置')}</h2>
        <button className={styles.iconButton} aria-label={text(locale, 'Close settings', '关闭设置')} onClick={onClose}><X size={19} /></button>
      </header>
      <nav className={styles.categories} aria-label={text(locale, 'Settings categories', '设置分类')}>
          {categories.map(({ value, icon: Icon, en, zh }) => <button key={value}
            aria-current={section === value ? 'page' : undefined}
            onClick={() => setSection(value)}><Icon size={17} />{text(locale, en, zh)}</button>)}
        </nav>
      <div className={styles.body}>
        <div className={styles.content}>
          {section === 'appearance' && <>
            <div className={styles.sectionHeading}><h3>{text(locale, 'Theme', '界面主题')}</h3></div>
            <div className={styles.themeOptions}>{THEMES.map(option => <button key={option.value}
              className={styles.themeOption} aria-pressed={theme === option.value}
              onClick={() => onThemeChange(option.value)}>
              <ThemePreview theme={option.value} />
              <span className={styles.themeLabel}>{text(locale, option.en, option.zh)}
                <span className={styles.selectedMark}>{theme === option.value && <Check size={13} strokeWidth={2.5} />}</span>
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
        </div>
      </div>
    </aside>
  </div>, document.body)
}
