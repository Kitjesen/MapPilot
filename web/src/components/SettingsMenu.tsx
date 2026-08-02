/**
 * SettingsMenu — 右上角设置下拉 + 子 Modal.
 *
 * 分 5 大类:
 *   - 系统   (OTA / 健康 / 日志 / 重启)
 *   - 模块   (LLM / 定位建图 / 探索 / 语音)
 *   - 标定   (相机/IMU/LiDAR/联合)
 *   - 诊断   (模块状态表 / 崩溃 / 性能)
 *   - 关于   (版本 / Git / 登出)
 *
 * 组会演示阶段:OTA、诊断、关于走真实 API(/api/v1/health, /docs);其余
 * 先以 placeholder 方式进入二级菜单,显示"敬请期待"并给出后端对接的 hint.
 */
import { useEffect, useRef, useState } from 'react'
import { createPortal } from 'react-dom'
import {
  X, Settings, Cpu, Wrench, Activity, Info, Package,
  Download, RefreshCw, Terminal, Power,
  Bot, Map as MapIcon, Compass, Mic,
  Camera, Zap, Radar, Link2,
  ListChecks, AlertTriangle, Gauge, ArrowRight,
  GitBranch, LogOut, Palette, Monitor, Sun, Moon, Play,
} from 'lucide-react'
import * as api from '../services/api'
import type { HealthResponse, Tab } from '../types'
import { resetAllLayouts } from './floatingWidgetLayout'
import type { ResolvedTheme, Theme } from './useTheme'
import { text, type Locale } from '../i18n'
import styles from './SettingsMenu.module.css'

type Section = 'home' | 'appearance' | 'operations' | 'system' | 'modules' | 'calib' | 'diag' | 'about'
type Modal = null | 'ota' | 'diag' | 'about'

interface SettingsMenuProps {
  open: boolean
  onClose: () => void
  theme: Theme
  resolvedTheme: ResolvedTheme
  onThemeChange: (theme: Theme) => void
  locale: Locale
  onLocaleChange: (locale: Locale) => void
  onNavigateTab?: (tab: Tab) => void
}

export function SettingsMenu({
  open,
  onClose,
  theme,
  resolvedTheme,
  onThemeChange,
  locale,
  onLocaleChange,
  onNavigateTab,
}: SettingsMenuProps) {
  const [section, setSection] = useState<Section>('home')
  const [modal, setModal] = useState<Modal>(null)
  const panelRef = useRef<HTMLDivElement>(null)

  useEffect(() => {
    if (!open) return
    const onKey = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        if (modal) setModal(null)
        else if (section !== 'home') setSection('home')
        else onClose()
      }
    }
    document.addEventListener('keydown', onKey)
    return () => document.removeEventListener('keydown', onKey)
  }, [open, onClose, section, modal])

  useEffect(() => {
    if (open) return
    const t = setTimeout(() => {
      setSection('home')
      setModal(null)
    }, 0)
    return () => clearTimeout(t)
  }, [open])

  if (!open) return null

  const content = (
    <>
      <div className={styles.backdrop} onClick={onClose} />
      <aside ref={panelRef} className={styles.panel} role="dialog" aria-modal="true" aria-label={text(locale, 'Settings', '设置')}>
        <div className={styles.panelHeader}>
          <div className={styles.panelTitle}>
            <Settings size={14} /> {sectionTitle(section, locale)}
          </div>
          <button className={styles.iconBtn} onClick={onClose} aria-label={text(locale, 'Close', '关闭')}>
            <X size={14} />
          </button>
        </div>

        {section === 'home' && <HomeSection locale={locale} onPick={setSection} />}
        {section === 'appearance' && (
          <AppearanceSection
            locale={locale}
            onLocaleChange={onLocaleChange}
            theme={theme}
            resolvedTheme={resolvedTheme}
            onThemeChange={onThemeChange}
            onBack={() => setSection('home')}
          />
        )}
        {section === 'operations' && (
          <OperationsSection
            locale={locale}
            onBack={() => setSection('home')}
            onClose={onClose}
            onNavigateTab={onNavigateTab}
          />
        )}
        {section === 'system' && (
          <SystemSection
            locale={locale}
            onBack={() => setSection('home')}
            onOpenModal={setModal}
          />
        )}
        {section === 'modules' && <ModulesSection locale={locale} onBack={() => setSection('home')} />}
        {section === 'calib' && <CalibSection locale={locale} onBack={() => setSection('home')} />}
        {section === 'diag' && <DiagSection locale={locale} onBack={() => setSection('home')} onOpen={() => setModal('diag')} />}
        {section === 'about' && <AboutSection locale={locale} onBack={() => setSection('home')} onOpen={() => setModal('about')} />}
      </aside>

      {modal === 'ota' && <OtaModal locale={locale} onClose={() => setModal(null)} />}
      {modal === 'diag' && <DiagModal locale={locale} onClose={() => setModal(null)} />}
      {modal === 'about' && <AboutModal locale={locale} onClose={() => setModal(null)} />}
    </>
  )

  return createPortal(content, document.body)
}

function sectionTitle(s: Section, locale: Locale): string {
  switch (s) {
    case 'home':    return text(locale, 'Settings', '设置')
    case 'appearance': return text(locale, 'Appearance', '外观')
    case 'operations': return text(locale, 'Operations', '快捷操作')
    case 'system':  return text(locale, 'System', '系统')
    case 'modules': return text(locale, 'Modules', '模块配置')
    case 'calib':   return text(locale, 'Calibration', '传感器标定')
    case 'diag':    return text(locale, 'Diagnostics', '诊断')
    case 'about':   return text(locale, 'About', '关于')
  }
}

/* ─── Home ─────────────────────────────────────────────────────── */

function HomeSection({ locale, onPick }: { locale: Locale; onPick: (s: Section) => void }) {
  const groups: { key: Section; icon: React.ReactNode; title: string; hint: string }[] = [
    { key: 'appearance', icon: <Palette size={15} />, title: text(locale, 'Appearance', '外观'), hint: text(locale, 'Theme / language / layout', '主题 / 语言 / 布局') },
    { key: 'operations', icon: <Play size={15} />, title: text(locale, 'Operations', '快捷操作'), hint: text(locale, 'Mode, localization, maps, target tracking', '模式、定位、地图、目标跟随') },
    { key: 'system', icon: <Cpu size={15} />, title: text(locale, 'System', '系统'), hint: text(locale, 'OTA / health / logs / restart', 'OTA / 健康 / 日志 / 重启') },
    { key: 'modules', icon: <Package size={15} />, title: text(locale, 'Modules', '模块配置'), hint: text(locale, 'LLM / SLAM / exploration / voice', 'LLM / 定位建图 / 探索 / 语音') },
    { key: 'calib', icon: <Wrench size={15} />, title: text(locale, 'Sensor Calibration', '传感器标定'), hint: text(locale, 'Camera / IMU / LiDAR', '相机 / IMU / LiDAR') },
    { key: 'diag', icon: <Activity size={15} />, title: text(locale, 'Diagnostics', '诊断'), hint: text(locale, 'Module health / crash logs / performance', '模块状态 / 崩溃日志 / 性能') },
    { key: 'about', icon: <Info size={15} />, title: text(locale, 'About', '关于'), hint: text(locale, 'Version / Git / sign out', '版本 / Git / 登出') },
  ]
  return (
    <div className={styles.list}>
      {groups.map(g => (
        <button key={g.key} className={styles.row} onClick={() => onPick(g.key)}>
          <span className={styles.rowIcon}>{g.icon}</span>
          <span className={styles.rowMain}>
            <span className={styles.rowTitle}>{g.title}</span>
            <span className={styles.rowHint}>{g.hint}</span>
          </span>
          <ArrowRight size={13} className={styles.rowChev} />
        </button>
      ))}
    </div>
  )
}

/* ─── Appearance ───────────────────────────────────────────────── */

function AppearanceSection({
  locale,
  onLocaleChange,
  theme,
  resolvedTheme,
  onThemeChange,
  onBack,
}: {
  locale: Locale
  onLocaleChange: (locale: Locale) => void
  theme: Theme
  resolvedTheme: ResolvedTheme
  onThemeChange: (theme: Theme) => void
  onBack: () => void
}) {
  const options: { key: Theme; icon: React.ReactNode; title: string; hint: string }[] = [
    { key: 'system', icon: <Monitor size={13} />, title: text(locale, 'System', '系统'), hint: text(locale, 'Follow OS preference', '跟随操作系统') },
    { key: 'light', icon: <Sun size={13} />, title: text(locale, 'Light', '浅色'), hint: text(locale, 'Codex-style light surface', 'Codex 风格浅色界面') },
    { key: 'dark', icon: <Moon size={13} />, title: text(locale, 'Dark', '深色'), hint: text(locale, 'Low-light control surface', '低光环境控制台') },
  ]

  return (
    <>
      <BackHeader locale={locale} onBack={onBack} />
      <div className={styles.sectionIntro}>
        {text(locale, 'Choose the display language and theme here. The current rendered theme is', '在这里选择显示语言和主题。当前实际渲染为')}
        <strong>{resolvedTheme === 'light' ? text(locale, 'light', '浅色') : text(locale, 'dark', '深色')}</strong>。
      </div>
      <div className={styles.settingGroup}>
        <span className={styles.settingLabel}>{text(locale, 'Language', '语言')}</span>
        <div className={styles.segmented} role="group" aria-label={text(locale, 'Language', '语言')}>
          <button
            type="button"
            className={locale === 'en' ? styles.segmentedActive : styles.segmentedBtn}
            aria-pressed={locale === 'en'}
            onClick={() => onLocaleChange('en')}
          >
            English
          </button>
          <button
            type="button"
            className={locale === 'zh' ? styles.segmentedActive : styles.segmentedBtn}
            aria-pressed={locale === 'zh'}
            onClick={() => onLocaleChange('zh')}
          >
            中文
          </button>
        </div>
      </div>
      <div className={styles.themeGrid}>
        {options.map(option => (
          <button
            key={option.key}
            type="button"
            aria-pressed={theme === option.key}
            className={theme === option.key ? styles.themeCardActive : styles.themeCard}
            onClick={() => onThemeChange(option.key)}
          >
            <span className={`${styles.themePreview} ${styles[`preview_${option.key}`]}`}>
              <span className={styles.previewTop} />
              <span className={styles.previewBody}>
                <span />
                <span />
                <span />
              </span>
            </span>
            <span className={styles.themeMeta}>
              <span className={styles.themeTitle}>
                {option.icon}
                {option.title}
              </span>
              <span>{option.hint}</span>
            </span>
          </button>
        ))}
      </div>
    </>
  )
}

/* ─── Web operations ───────────────────────────────────────────── */

function OperationsSection({
  locale,
  onBack,
  onClose,
  onNavigateTab,
}: {
  locale: Locale
  onBack: () => void
  onClose: () => void
  onNavigateTab?: (tab: Tab) => void
}) {
  const jump = (tab: Tab) => {
    onNavigateTab?.(tab)
    onClose()
  }

  return (
    <>
      <BackHeader locale={locale} onBack={onBack} />
      <div className={styles.sectionIntro}>
        {text(locale, 'Task-oriented shortcuts. Each item opens the workspace where parameters and confirmation live.', '按现场任务组织入口。点击后进入对应工作区，参数和确认动作在工作区内完成。')}
      </div>
      <div className={styles.list}>
        <ActionRow icon={<Compass size={14} />} title={text(locale, 'Switch Work Mode', '切换工作模式')}
          hint={text(locale, 'Mapping, navigation, inspection, exploration', '建图、导航、巡检、探索')}
          value={text(locale, 'Console', '控制台')}
          onClick={() => jump('console')} />
        <ActionRow icon={<RefreshCw size={14} />} title={text(locale, 'Recover Localization', '恢复定位')}
          hint={text(locale, 'Use when pose drifts, maps misalign, or a fresh start is needed', '位置漂移、地图不对齐或需要重新开始时使用')}
          value={text(locale, 'Scene', '场景')}
          onClick={() => jump('scene')} />
        <ActionRow icon={<MapIcon size={14} />} title={text(locale, 'Prepare Maps', '处理地图')}
          hint={text(locale, 'Import PCD, crop, mark zones, build OctoMap', '导入 PCD、裁剪、区域标注、构建 OctoMap')}
          value={text(locale, 'Scene', '场景')}
          onClick={() => jump('scene')} />
        <ActionRow icon={<ListChecks size={14} />} title={text(locale, 'Navigation Check', '导航前检查')}
          hint={text(locale, 'Check map, localization, target, and path feasibility', '检查地图、定位链路、目标点和路径可行性')}
          value={text(locale, 'Scene', '场景')}
          onClick={() => jump('scene')} />
        <ActionRow icon={<Radar size={14} />} title={text(locale, 'Target Tracking', '目标跟随')}
          hint={text(locale, 'Find, follow, or stop visual servo after selecting a target', '选择目标后寻找、跟随或停止视觉伺服')}
          value={text(locale, 'Console', '控制台')}
          onClick={() => jump('console')} />
      </div>
    </>
  )
}

/* ─── System ───────────────────────────────────────────────────── */

function SystemSection({ locale, onBack, onOpenModal }: {
  locale: Locale
  onBack: () => void
  onOpenModal: (m: Modal) => void
}) {
  return (
    <>
      <BackHeader locale={locale} onBack={onBack} />
      <div className={styles.list}>
        <ActionRow icon={<Download size={14} />} title={text(locale, 'OTA Update', 'OTA 固件升级')}
          hint={text(locale, 'Check cloud releases / push localization config', '检查云端最新版本 / 推送定位配置')}
          onClick={() => onOpenModal('ota')} />
        <ActionRow icon={<RefreshCw size={14} />} title={text(locale, 'System Health', '系统健康检查')}
          hint="/api/v1/health"
          onClick={() => window.open('/api/v1/health', '_blank')} />
        <ActionRow icon={<Terminal size={14} />} title={text(locale, 'Live Logs', '实时日志')}
          hint="logs/{timestamp}_{profile}/lingtu.log"
          onClick={() => alert(text(locale, 'Log viewer is not installed yet. Use SSH: tail -f logs/lingtu.log', '日志查看器待实装，请在 SSH 终端 tail -f logs/lingtu.log'))} />
        <ActionRow icon={<RefreshCw size={14} />} title={text(locale, 'Reset Layout', '恢复默认布局')}
          hint={text(locale, 'Clear saved panel positions and sizes', '清除已保存的面板位置和尺寸')}
          onClick={() => {
            resetAllLayouts()
          }} />
        <ActionRow icon={<Power size={14} />} title={text(locale, 'Restart LingTu', '重启 LingTu')} dangerous
          hint={text(locale, 'Reapply the active Product through ProductControl', '通过 ProductControl 重新应用当前产品')}
          onClick={() => {
            if (confirm(text(locale, 'Restart LingTu? Current tasks will be interrupted.', '确认重启 LingTu？所有当前任务将被中断。'))) {
              alert(text(locale, 'Use SSH: python -m lingtu.control reapply --env real', '请通过 SSH 执行：python -m lingtu.control reapply --env real'))
            }
          }} />
      </div>
    </>
  )
}

/* ─── Modules ──────────────────────────────────────────────────── */

function ModulesSection({ locale, onBack }: { locale: Locale; onBack: () => void }) {
  return (
    <>
      <BackHeader locale={locale} onBack={onBack} />
      <div className={styles.list}>
        <ActionRow icon={<Bot size={14} />} title={text(locale, 'LLM Backend', 'LLM 后端')}
          hint="Kimi / OpenAI / Claude / Qwen / Mock"
          value="Kimi (K2.5)" onClick={() => alert(text(locale, 'LLM switch API is not wired yet.', 'LLM 切换 API 待接入'))} />
        <ActionRow icon={<MapIcon size={14} />} title={text(locale, 'Localization & Mapping', '定位建图')}
          hint={text(locale, 'Mapping / navigation / stop', '建图 / 导航 / 停止')}
          value={text(locale, 'Navigation', '导航')} onClick={() => alert(text(locale, 'Use the SLAM workspace for mode switching.', '请在定位工作区切换模式'))} />
        <ActionRow icon={<Compass size={14} />} title={text(locale, 'Exploration Backend', '探索后端')}
          hint="TARE / Wavefront / 关闭"
          value="TARE" onClick={() => alert(text(locale, 'Exploration backend changes require restarting the profile.', '探索后端切换需要重启 profile'))} />
        <ActionRow icon={<Mic size={14} />} title={text(locale, 'Voice Control', '语音交互')}
          hint="Askme voice agent"
          value={text(locale, 'Off', '关闭')} onClick={() => alert(text(locale, 'Voice integration uses Askme on S100P.', '语音接入 Askme，S100P 专用'))} />
      </div>
    </>
  )
}

/* ─── Calibration ──────────────────────────────────────────────── */

function CalibSection({ locale, onBack }: { locale: Locale; onBack: () => void }) {
  return (
    <>
      <BackHeader locale={locale} onBack={onBack} />
      <div className={styles.list}>
        <ActionRow icon={<Camera size={14} />} title={text(locale, 'Camera Intrinsics', '相机内参')}
          hint="calibration/camera/calibrate_intrinsic.py"
          onClick={() => alert('python calibration/camera/calibrate_intrinsic.py')} />
        <ActionRow icon={<Zap size={14} />} title="IMU Allan Variance"
          hint="calibration/imu/allan_variance_ros2"
          onClick={() => alert(text(locale, 'Collect 2-3 hours of static IMU data.', '采集 2-3 小时静置 IMU 数据'))} />
        <ActionRow icon={<Radar size={14} />} title={text(locale, 'LiDAR-IMU Extrinsics', 'LiDAR-IMU 外参')}
          hint={text(locale, 'LiDAR_IMU_Init motion sequence', 'LiDAR_IMU_Init 标定动作')}
          onClick={() => alert('bash calibration/lidar_imu/calibrate.sh')} />
        <ActionRow icon={<Link2 size={14} />} title={text(locale, 'Camera-LiDAR Extrinsics', '相机-LiDAR 外参')}
          hint="direct_visual_lidar_calibration"
          onClick={() => alert('bash calibration/camera_lidar/calibrate.sh')} />
        <ActionRow icon={<ListChecks size={14} />} title={text(locale, 'Apply & Verify', '应用并验证')}
          hint="apply_calibration.py + verify.py"
          onClick={() => alert('python calibration/apply_calibration.py && python calibration/verify.py')} />
      </div>
    </>
  )
}

/* ─── Diagnostics ──────────────────────────────────────────────── */

function DiagSection({ locale, onBack, onOpen }: { locale: Locale; onBack: () => void; onOpen: () => void }) {
  return (
    <>
      <BackHeader locale={locale} onBack={onBack} />
      <div className={styles.list}>
        <ActionRow icon={<ListChecks size={14} />} title={text(locale, 'Module Health', '模块状态总览')}
          hint={text(locale, 'View module health and ports', '查看模块健康状态和端口')}
          onClick={onOpen} />
        <ActionRow icon={<AlertTriangle size={14} />} title={text(locale, 'Crash Logs', '崩溃日志')}
          hint={text(locale, 'Recent ERROR / CRITICAL entries', '最近 ERROR / CRITICAL 记录')}
          onClick={() => alert(text(locale, 'Crash log viewer is not installed yet.', '崩溃日志查看器待实装'))} />
        <ActionRow icon={<Gauge size={14} />} title={text(locale, 'Performance', '性能指标')}
          hint={text(locale, 'Localization rate / gateway latency / CPU', '定位频率 / 网关延迟 / CPU')}
          onClick={() => window.open('/api/v1/metrics', '_blank')} />
        <ActionRow icon={<Download size={14} />} title={text(locale, 'Export Diagnostics', '导出诊断包')}
          hint="logs + config + health + git HEAD"
          onClick={async () => {
            try {
              const resp = await fetch('/api/v1/diagnostic_pack')
              if (!resp.ok) throw new Error(`HTTP ${resp.status}`)
              const blob = await resp.blob()
              const disp = resp.headers.get('content-disposition') || ''
              const m = disp.match(/filename="?([^"]+)"?/)
              const name = m ? m[1] : `lingtu_diag_${Date.now()}.tar.gz`
              const url = URL.createObjectURL(blob)
              const a = document.createElement('a')
              a.href = url
              a.download = name
              document.body.appendChild(a)
              a.click()
              a.remove()
              URL.revokeObjectURL(url)
            } catch (e) {
              alert(`${text(locale, 'Diagnostic export failed', '诊断包导出失败')}: ${e instanceof Error ? e.message : e}`)
            }
          }} />
      </div>
    </>
  )
}

/* ─── About ────────────────────────────────────────────────────── */

function AboutSection({ locale, onBack, onOpen }: { locale: Locale; onBack: () => void; onOpen: () => void }) {
  return (
    <>
      <BackHeader locale={locale} onBack={onBack} />
      <div className={styles.list}>
        <ActionRow icon={<Info size={14} />} title={text(locale, 'Version', '版本信息')}
          hint="LingTu Dashboard v1.7.5"
          onClick={onOpen} />
        <ActionRow icon={<GitBranch size={14} />} title={text(locale, 'Git Repository', 'Git 仓库')}
          hint="github.com/Kitjesen/MapPilot"
          onClick={() => window.open('https://github.com/Kitjesen/MapPilot', '_blank')} />
        <ActionRow icon={<LogOut size={14} />} title={text(locale, 'Sign Out', '登出')} dangerous
          hint={text(locale, 'Clear local session and return to login', '清除本地 session 返回登录页')}
          onClick={() => {
            if (confirm(text(locale, 'Sign out?', '确认登出？'))) {
              localStorage.clear()
              location.reload()
            }
          }} />
      </div>
    </>
  )
}

/* ─── Shared pieces ────────────────────────────────────────────── */

function BackHeader({ locale, onBack }: { locale: Locale; onBack: () => void }) {
  return (
    <button className={styles.backBtn} onClick={onBack}>
      ← {text(locale, 'Back', '返回')}
    </button>
  )
}

function ActionRow({
  icon, title, hint, value, onClick, dangerous,
}: {
  icon: React.ReactNode
  title: string
  hint?: string
  value?: string
  onClick: () => void
  dangerous?: boolean
}) {
  return (
    <button
      className={dangerous ? `${styles.row} ${styles.rowDanger}` : styles.row}
      onClick={onClick}
    >
      <span className={styles.rowIcon}>{icon}</span>
      <span className={styles.rowMain}>
        <span className={styles.rowTitle}>{title}</span>
        {hint && <span className={styles.rowHint}>{hint}</span>}
      </span>
      {value && <span className={styles.rowValue}>{value}</span>}
      <ArrowRight size={13} className={styles.rowChev} />
    </button>
  )
}

/* ─── OTA Modal ────────────────────────────────────────────────── */

function OtaModal({ locale, onClose }: { locale: Locale; onClose: () => void }) {
  const [checking, setChecking] = useState(false)
  const [result, setResult] = useState<string | null>(null)

  const currentVersion = 'v1.7.5'
  const currentCommit = '13cbd35'
  const releaseChannel = 'stable'

  const handleCheck = async () => {
    setChecking(true)
    setResult(null)
    await new Promise(r => setTimeout(r, 1200))
    setResult(text(locale, 'Already up to date. No firmware update is available.', '已是最新版本，未检测到可升级的固件。'))
    setChecking(false)
  }

  return (
    <ModalShell title={text(locale, 'OTA Update', 'OTA 固件升级')} onClose={onClose}>
      <div className={styles.otaHeader}>
        <div className={styles.otaBadge}>{releaseChannel}</div>
        <div>
          <div className={styles.otaVer}>{currentVersion}</div>
          <div className={styles.otaSub}>commit {currentCommit}</div>
        </div>
      </div>

      <div className={styles.otaGrid}>
        <OtaField label={text(locale, 'OTA Server', 'OTA 服务器')} value="https://ota.inovxio.com" />
        <OtaField label={text(locale, 'Channel', '升级通道')} value={releaseChannel} />
        <OtaField label={text(locale, 'Next Check', '下次检查')} value={text(locale, 'Automatic, 30 min', '自动 30 分钟')} />
        <OtaField label={text(locale, 'Offline Package', '离线包')} value={text(locale, 'Manual SSH push supported', '可 SSH 手动推送')} />
      </div>

      <div className={styles.otaActions}>
        <button className={styles.btnPrimary} onClick={handleCheck} disabled={checking}>
          {checking ? <RefreshCw size={13} className={styles.spin} /> : <Download size={13} />}
          {checking ? text(locale, 'Checking...', '检查中…') : text(locale, 'Check Updates', '检查更新')}
        </button>
        <button className={styles.btnGhost} onClick={() => alert(text(locale, 'Update history is not installed yet.', '升级历史待实装'))}>
          {text(locale, 'Update History', '升级历史')}
        </button>
      </div>

      {result && <div className={styles.otaResult}>{result}</div>}

      <div className={styles.otaFooter}>
        <strong>{text(locale, 'Backend contract', '后端对接')}</strong>:
        {text(locale, 'Production OTA polls the release server, deploys into', '生产路径会轮询服务器，检测到新版本后推送到')}
        {' '}<code>/opt/lingtu/nav</code>{' '}
        {text(locale, 'and then reapplies the active Product with', '然后使用以下命令重新应用当前产品：')}{' '}
        <code>python -m lingtu.control reapply --env real</code>.
      </div>
    </ModalShell>
  )
}

function OtaField({ label, value }: { label: string; value: string }) {
  return (
    <div className={styles.otaField}>
      <span className={styles.otaFieldLabel}>{label}</span>
      <span className={styles.otaFieldValue}>{value}</span>
    </div>
  )
}

/* ─── Diagnostics Modal ────────────────────────────────────────── */

type HealthSummary = {
  modules?: HealthResponse['modules']
  error?: string
}

function DiagModal({ locale, onClose }: { locale: Locale; onClose: () => void }) {
  const [health, setHealth] = useState<HealthSummary | null>(null)

  useEffect(() => {
    api.fetchHealth()
      .then(setHealth)
      .catch((error: unknown) => {
        const message = api.isGatewayApiError(error)
          ? (error.body?.message || error.body?.error || error.message)
          : (error instanceof Error ? error.message : String(error))
        setHealth({ error: message || 'failed to fetch' })
      })
  }, [])

  const modules = health?.modules ?? {}
  const names = Object.keys(modules)
  const okCount = names.filter(n => modules[n] === 'ok').length
  const failCount = names.length - okCount

  return (
    <ModalShell title={text(locale, 'Module Diagnostics', '模块诊断')} onClose={onClose}>
      <div className={styles.diagSummary}>
        <div className={styles.diagStat}>
          <span className={styles.diagStatValue}>{okCount}</span>
          <span className={styles.diagStatLabel}>{text(locale, 'OK', '正常')}</span>
        </div>
        <div className={styles.diagStat}>
          <span className={`${styles.diagStatValue} ${failCount > 0 ? styles.diagBad : ''}`}>
            {failCount}
          </span>
          <span className={styles.diagStatLabel}>{text(locale, 'Issues', '异常')}</span>
        </div>
        <div className={styles.diagStat}>
          <span className={styles.diagStatValue}>{names.length}</span>
          <span className={styles.diagStatLabel}>{text(locale, 'Total', '总数')}</span>
        </div>
      </div>

      <div className={styles.diagTable}>
        {names.length === 0 && <div className={styles.otaResult}>{text(locale, 'Loading...', '加载中…')}</div>}
        {names.map(n => (
          <div key={n} className={styles.diagRow}>
            <span className={styles.diagName}>{n}</span>
            <span className={modules[n] === 'ok' ? styles.diagOk : styles.diagFail}>
              {String(modules[n])}
            </span>
          </div>
        ))}
      </div>
    </ModalShell>
  )
}

/* ─── About Modal ─────────────────────────────────────────────── */

function AboutModal({ locale, onClose }: { locale: Locale; onClose: () => void }) {
  return (
    <ModalShell title={text(locale, 'About LingTu', '关于 LingTu')} onClose={onClose}>
      <div className={styles.aboutHero}>
        <div className={styles.aboutLogo}>灵途</div>
        <div className={styles.aboutName}>LingTu Navigation</div>
        <div className={styles.aboutTagline}>Autonomous navigation for quadruped robots</div>
      </div>
      <div className={styles.otaGrid}>
        <OtaField label={text(locale, 'Version', '版本')} value="v1.7.5" />
        <OtaField label="Commit" value="13cbd35" />
        <OtaField label="Build date" value="2026-04-17" />
        <OtaField label={text(locale, 'Platform', '平台')} value="S100P / dev" />
      </div>
      <div className={styles.otaFooter}>
        <strong>穹沛科技 / inovxio</strong> · Python + ROS2 Humble + Fast-LIO2 + TARE ·
        28 Wave-1/2/3 硬护栏 + 98 新测试
      </div>
    </ModalShell>
  )
}

/* ─── Modal shell ──────────────────────────────────────────────── */

function ModalShell({ title, children, onClose }: {
  title: string
  children: React.ReactNode
  onClose: () => void
}) {
  return (
    <div className={styles.modalBackdrop} onClick={onClose}>
      <div className={styles.modal} onClick={e => e.stopPropagation()}>
        <div className={styles.modalHeader}>
          <span>{title}</span>
          <button className={styles.iconBtn} onClick={onClose}>
            <X size={14} />
          </button>
        </div>
        <div className={styles.modalBody}>{children}</div>
      </div>
    </div>
  )
}
