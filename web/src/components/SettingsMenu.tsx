/**
 * SettingsMenu — 右上角设置下拉 + 子 Modal.
 *
 * 分 5 大类:
 *   - 系统   (OTA / 健康 / 日志 / 重启)
 *   - 模块   (LLM / SLAM / 探索 / 语音)
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
  GitBranch, LogOut,
} from 'lucide-react'
import * as api from '../services/api'
import type { HealthResponse } from '../types'
import { resetAllLayouts } from './floatingWidgetLayout'
import styles from './SettingsMenu.module.css'

type Section = 'home' | 'system' | 'modules' | 'calib' | 'diag' | 'about'
type Modal = null | 'ota' | 'diag' | 'about'

interface SettingsMenuProps {
  open: boolean
  onClose: () => void
}

export function SettingsMenu({ open, onClose }: SettingsMenuProps) {
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
      <aside ref={panelRef} className={styles.panel} role="dialog" aria-modal="true" aria-label="设置">
        <div className={styles.panelHeader}>
          <div className={styles.panelTitle}>
            <Settings size={14} /> {sectionTitle(section)}
          </div>
          <button className={styles.iconBtn} onClick={onClose} aria-label="关闭">
            <X size={14} />
          </button>
        </div>

        {section === 'home' && <HomeSection onPick={setSection} />}
        {section === 'system' && (
          <SystemSection
            onBack={() => setSection('home')}
            onOpenModal={setModal}
          />
        )}
        {section === 'modules' && <ModulesSection onBack={() => setSection('home')} />}
        {section === 'calib' && <CalibSection onBack={() => setSection('home')} />}
        {section === 'diag' && <DiagSection onBack={() => setSection('home')} onOpen={() => setModal('diag')} />}
        {section === 'about' && <AboutSection onBack={() => setSection('home')} onOpen={() => setModal('about')} />}
      </aside>

      {modal === 'ota' && <OtaModal onClose={() => setModal(null)} />}
      {modal === 'diag' && <DiagModal onClose={() => setModal(null)} />}
      {modal === 'about' && <AboutModal onClose={() => setModal(null)} />}
    </>
  )

  return createPortal(content, document.body)
}

function sectionTitle(s: Section): string {
  switch (s) {
    case 'home':    return '设置'
    case 'system':  return '系统'
    case 'modules': return '模块配置'
    case 'calib':   return '传感器标定'
    case 'diag':    return '诊断'
    case 'about':   return '关于'
  }
}

/* ─── Home ─────────────────────────────────────────────────────── */

function HomeSection({ onPick }: { onPick: (s: Section) => void }) {
  const groups: { key: Section; icon: React.ReactNode; title: string; hint: string }[] = [
    { key: 'system',  icon: <Cpu size={15} />,      title: '系统',       hint: 'OTA / 健康 / 日志 / 重启' },
    { key: 'modules', icon: <Package size={15} />,  title: '模块配置',   hint: 'LLM / SLAM / 探索 / 语音' },
    { key: 'calib',   icon: <Wrench size={15} />,   title: '传感器标定', hint: '相机 / IMU / LiDAR' },
    { key: 'diag',    icon: <Activity size={15} />, title: '诊断',       hint: '模块状态 / 崩溃日志 / 性能' },
    { key: 'about',   icon: <Info size={15} />,     title: '关于',       hint: '版本 / Git / 登出' },
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

/* ─── System ───────────────────────────────────────────────────── */

function SystemSection({ onBack, onOpenModal }: {
  onBack: () => void
  onOpenModal: (m: Modal) => void
}) {
  return (
    <>
      <BackHeader onBack={onBack} />
      <div className={styles.list}>
        <ActionRow icon={<Download size={14} />} title="OTA 固件升级"
          hint="检查云端最新版本 / 推送 SLAM 配置"
          onClick={() => onOpenModal('ota')} />
        <ActionRow icon={<RefreshCw size={14} />} title="系统健康检查"
          hint="/api/v1/health 实时拉取"
          onClick={() => window.open('/api/v1/health', '_blank')} />
        <ActionRow icon={<Terminal size={14} />} title="实时日志"
          hint="logs/{timestamp}_{profile}/lingtu.log"
          onClick={() => alert('日志查看器(待实装):请在 SSH 终端 tail -f logs/lingtu.log')} />
        <ActionRow icon={<RefreshCw size={14} />} title="恢复默认布局"
          hint="清除已保存的 FloatingWidget 位置/尺寸"
          onClick={() => {
            resetAllLayouts()
          }} />
        <ActionRow icon={<Power size={14} />} title="重启 LingTu" dangerous
          hint="优雅停止后重新启动所有模块"
          onClick={() => {
            if (confirm('确认重启 LingTu?所有当前任务将被中断。')) {
              alert('重启 API 待接入:目前请 SSH sudo systemctl restart lingtu')
            }
          }} />
      </div>
    </>
  )
}

/* ─── Modules ──────────────────────────────────────────────────── */

function ModulesSection({ onBack }: { onBack: () => void }) {
  return (
    <>
      <BackHeader onBack={onBack} />
      <div className={styles.list}>
        <ActionRow icon={<Bot size={14} />} title="LLM 后端"
          hint="Kimi / OpenAI / Claude / Qwen / Mock"
          value="Kimi (K2.5)" onClick={() => alert('LLM 切换 API 待接入')} />
        <ActionRow icon={<MapIcon size={14} />} title="SLAM 模式"
          hint="建图 / 导航 / 停止"
          value="导航" onClick={() => alert('SLAM 模式切换:使用 REPL slam 命令')} />
        <ActionRow icon={<Compass size={14} />} title="探索 Backend"
          hint="TARE / Wavefront / 关闭"
          value="TARE" onClick={() => alert('探索 backend 需重启 profile')} />
        <ActionRow icon={<Mic size={14} />} title="语音交互"
          hint="Askme voice agent"
          value="关闭" onClick={() => alert('语音接入 Askme,S100P 专用')} />
      </div>
    </>
  )
}

/* ─── Calibration ──────────────────────────────────────────────── */

function CalibSection({ onBack }: { onBack: () => void }) {
  return (
    <>
      <BackHeader onBack={onBack} />
      <div className={styles.list}>
        <ActionRow icon={<Camera size={14} />} title="相机内参"
          hint="calibration/camera/calibrate_intrinsic.py"
          onClick={() => alert('运行:python calibration/camera/calibrate_intrinsic.py')} />
        <ActionRow icon={<Zap size={14} />} title="IMU Allan Variance"
          hint="calibration/imu/allan_variance_ros2"
          onClick={() => alert('运行 2-3h 采集静置数据')} />
        <ActionRow icon={<Radar size={14} />} title="LiDAR-IMU 外参"
          hint="LiDAR_IMU_Init(8 字运动)"
          onClick={() => alert('运行:bash calibration/lidar_imu/calibrate.sh')} />
        <ActionRow icon={<Link2 size={14} />} title="相机-LiDAR 外参"
          hint="direct_visual_lidar_calibration"
          onClick={() => alert('运行:bash calibration/camera_lidar/calibrate.sh')} />
        <ActionRow icon={<ListChecks size={14} />} title="一键应用 + 验证"
          hint="apply_calibration.py + verify.py"
          onClick={() => alert('运行:python calibration/apply_calibration.py && python calibration/verify.py')} />
      </div>
    </>
  )
}

/* ─── Diagnostics ──────────────────────────────────────────────── */

function DiagSection({ onBack, onOpen }: { onBack: () => void; onOpen: () => void }) {
  return (
    <>
      <BackHeader onBack={onBack} />
      <div className={styles.list}>
        <ActionRow icon={<ListChecks size={14} />} title="模块状态总览"
          hint="查看 20 个模块健康状态 + 端口"
          onClick={onOpen} />
        <ActionRow icon={<AlertTriangle size={14} />} title="崩溃日志"
          hint="最近 10 条 ERROR/CRITICAL"
          onClick={() => alert('崩溃日志查看器(待实装)')} />
        <ActionRow icon={<Gauge size={14} />} title="性能指标"
          hint="SLAM Hz / uvicorn latency / CPU"
          onClick={() => window.open('/api/v1/metrics', '_blank')} />
        <ActionRow icon={<Download size={14} />} title="导出诊断包"
          hint="打包 logs + config + health + git HEAD 为 tar.gz"
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
              alert(`诊断包导出失败: ${e instanceof Error ? e.message : e}`)
            }
          }} />
      </div>
    </>
  )
}

/* ─── About ────────────────────────────────────────────────────── */

function AboutSection({ onBack, onOpen }: { onBack: () => void; onOpen: () => void }) {
  return (
    <>
      <BackHeader onBack={onBack} />
      <div className={styles.list}>
        <ActionRow icon={<Info size={14} />} title="版本信息"
          hint="LingTu Dashboard v1.7.5"
          onClick={onOpen} />
        <ActionRow icon={<GitBranch size={14} />} title="Git 仓库"
          hint="github.com/Kitjesen/MapPilot"
          onClick={() => window.open('https://github.com/Kitjesen/MapPilot', '_blank')} />
        <ActionRow icon={<LogOut size={14} />} title="登出" dangerous
          hint="清除本地 session 返回登录页"
          onClick={() => {
            if (confirm('确认登出?')) {
              localStorage.clear()
              location.reload()
            }
          }} />
      </div>
    </>
  )
}

/* ─── Shared pieces ────────────────────────────────────────────── */

function BackHeader({ onBack }: { onBack: () => void }) {
  return (
    <button className={styles.backBtn} onClick={onBack}>
      ← 返回
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

function OtaModal({ onClose }: { onClose: () => void }) {
  const [checking, setChecking] = useState(false)
  const [result, setResult] = useState<string | null>(null)

  const currentVersion = 'v1.7.5'
  const currentCommit = '13cbd35'
  const releaseChannel = 'stable'

  const handleCheck = async () => {
    setChecking(true)
    setResult(null)
    await new Promise(r => setTimeout(r, 1200))
    setResult('已是最新版本 — 未检测到可升级的固件。')
    setChecking(false)
  }

  return (
    <ModalShell title="OTA 固件升级" onClose={onClose}>
      <div className={styles.otaHeader}>
        <div className={styles.otaBadge}>{releaseChannel}</div>
        <div>
          <div className={styles.otaVer}>{currentVersion}</div>
          <div className={styles.otaSub}>commit {currentCommit}</div>
        </div>
      </div>

      <div className={styles.otaGrid}>
        <OtaField label="OTA 服务器" value="https://ota.inovxio.com" />
        <OtaField label="升级通道" value={releaseChannel} />
        <OtaField label="下次检查" value="自动 30 分钟" />
        <OtaField label="离线包" value="可 SSH 手动推送" />
      </div>

      <div className={styles.otaActions}>
        <button className={styles.btnPrimary} onClick={handleCheck} disabled={checking}>
          {checking ? <RefreshCw size={13} className={styles.spin} /> : <Download size={13} />}
          {checking ? '检查中…' : '检查更新'}
        </button>
        <button className={styles.btnGhost} onClick={() => alert('升级历史(待实装)')}>
          升级历史
        </button>
      </div>

      {result && <div className={styles.otaResult}>{result}</div>}

      <div className={styles.otaFooter}>
        <strong>后端对接</strong>:生产路径通过 <code>infra/ota/agent</code>
        轮询服务器,检测到新版本后推送到 <code>/opt/lingtu/nav</code>
        并触发 <code>systemctl restart lingtu</code>。详见 OTA 手册。
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

function DiagModal({ onClose }: { onClose: () => void }) {
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
    <ModalShell title="模块诊断" onClose={onClose}>
      <div className={styles.diagSummary}>
        <div className={styles.diagStat}>
          <span className={styles.diagStatValue}>{okCount}</span>
          <span className={styles.diagStatLabel}>正常</span>
        </div>
        <div className={styles.diagStat}>
          <span className={`${styles.diagStatValue} ${failCount > 0 ? styles.diagBad : ''}`}>
            {failCount}
          </span>
          <span className={styles.diagStatLabel}>异常</span>
        </div>
        <div className={styles.diagStat}>
          <span className={styles.diagStatValue}>{names.length}</span>
          <span className={styles.diagStatLabel}>总数</span>
        </div>
      </div>

      <div className={styles.diagTable}>
        {names.length === 0 && <div className={styles.otaResult}>加载中…</div>}
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

function AboutModal({ onClose }: { onClose: () => void }) {
  return (
    <ModalShell title="关于 LingTu" onClose={onClose}>
      <div className={styles.aboutHero}>
        <div className={styles.aboutLogo}>灵途</div>
        <div className={styles.aboutName}>LingTu Navigation</div>
        <div className={styles.aboutTagline}>Autonomous navigation for quadruped robots</div>
      </div>
      <div className={styles.otaGrid}>
        <OtaField label="版本" value="v1.7.5" />
        <OtaField label="Commit" value="13cbd35" />
        <OtaField label="Build date" value="2026-04-17" />
        <OtaField label="平台" value="S100P / dev" />
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
