import { useEffect, useRef } from 'react'
import { ThemeToggle } from './ThemeToggle'
import { useTheme } from './useTheme'
import { CountUp } from './CountUp'
import styles from './Landing.module.css'

/* ── Data ── */

const HIGHLIGHTS = [
  {
    icon: '⚙',
    title: '全栈自研',
    desc: '从 FOC 电机驱动到 SLAM 建图，从语义导航到 OTA 云端——每一层都由穹沛自主研发，没有黑盒依赖。',
  },
  {
    icon: '🌿',
    title: '真实世界',
    desc: '不在实验室跑圈。泥地、楼梯、工地、山坡——LingTu 在人不想去的地方保持稳定。',
  },
  {
    icon: '⬡',
    title: 'Dashboard 控制',
    desc: 'Arc 风格 Web 控制台，实时相机画面、地图、点云三屏联动。一个浏览器看全局。',
  },
  {
    icon: '↑',
    title: 'OTA 一键部署',
    desc: '远程推送固件与模型，零停机热更新。从云端到机器人，更新链路全程可审计。',
  },
]

const LAYERS = [
  { l: 0, name: 'Safety', modules: 'SafetyRing · Geofence · CmdVelMux', tag: 'L0', desc: '安全反射 — 最高优先级，任何层都无法绕过' },
  { l: 1, name: 'Hardware', modules: 'ThunderDriver · CameraBridge · SLAM', tag: 'L1', desc: '硬件抽象 — 驱动、传感器、定位' },
  { l: 2, name: 'Maps', modules: 'OccupancyGrid · ESDF · ElevationMap', tag: 'L2', desc: '地图层 — 可通行性 + 高程 + 代价场' },
  { l: 3, name: 'Perception', modules: 'Detector · Encoder · SemanticMapper', tag: 'L3', desc: '感知层 — BPU 45ms 检测 + 场景图构建' },
  { l: 4, name: 'Decision', modules: 'SemanticPlanner · LLM · VisualServo', tag: 'L4', desc: '决策层 — 语义目标解析 + 视觉伺服' },
  { l: 5, name: 'Navigation', modules: 'NavigationModule · A* · WaypointTracker', tag: 'L5', desc: '规划层 — 全局路径 + 避障 + 任务 FSM' },
  { l: 6, name: 'Interface', modules: 'Gateway · MCP · Teleop', tag: 'L6', desc: '接口层 — HTTP/WS/SSE + AI Agent 控制' },
]

const STATS = [
  { end: 128, suffix: ' TOPS', label: 'Nash BPU 算力', note: '板载 AI 加速器' },
  { end: 45, suffix: 'ms', label: '目标检测延迟', note: 'YOLO11s-seg @ S100P' },
  { end: 32, suffix: '', label: '并行模块数', note: 'dev profile 实测' },
  { end: 10, suffix: 'Hz', label: 'SLAM 更新频率', note: 'Fast-LIO2 + ICP' },
]

/* ── Hooks ── */

function useReveal(selector: string) {
  useEffect(() => {
    const els = document.querySelectorAll<HTMLElement>(selector)
    const observer = new IntersectionObserver(
      (entries) => {
        entries.forEach((e) => {
          if (e.isIntersecting) {
            const el = e.target as HTMLElement
            const delay = el.dataset.delay ? parseInt(el.dataset.delay) : 0
            setTimeout(() => el.classList.add(styles.visible), delay)
            observer.unobserve(el)
          }
        })
      },
      { threshold: 0.12 }
    )
    els.forEach((el) => observer.observe(el))
    return () => observer.disconnect()
  }, [selector])
}

/* ── Component ── */

export function Landing() {
  const { theme, toggle } = useTheme()
  const layerRefs = useRef<(HTMLDivElement | null)[]>([])

  // Reveal scroll animations
  useReveal(`.${styles.reveal}`)

  // Layer stagger reveal
  useEffect(() => {
    const observer = new IntersectionObserver(
      (entries) => {
        entries.forEach((e) => {
          if (e.isIntersecting) {
            const idx = layerRefs.current.indexOf(e.target as HTMLDivElement)
            setTimeout(() => {
              ;(e.target as HTMLElement).classList.add(styles.visible)
            }, idx * 80)
            observer.unobserve(e.target)
          }
        })
      },
      { threshold: 0.08 }
    )
    layerRefs.current.forEach((el) => el && observer.observe(el))
    return () => observer.disconnect()
  }, [])

  return (
    <div className={styles.landing}>
      {/* Ambient background */}
      <div className={styles.orb + ' ' + styles.orb1} aria-hidden="true" />
      <div className={styles.orb + ' ' + styles.orb2} aria-hidden="true" />
      <div className={styles.orb + ' ' + styles.orb3} aria-hidden="true" />
      <div className={styles.gridOverlay} aria-hidden="true" />

      {/* Nav */}
      <nav className={styles.nav} role="navigation" aria-label="主导航">
        <a href="#" className={styles.navLogo} aria-label="LingTu 灵途">
          <div className={styles.navLogoMark} aria-hidden="true">灵</div>
          <div>
            <div className={styles.navLogoText}>LingTu</div>
            <div className={styles.navLogoSub}>灵途导航系统</div>
          </div>
        </a>
        <div className={styles.navRight}>
          <a href="#highlights" className={styles.navLink}>产品</a>
          <a href="#architecture" className={styles.navLink}>架构</a>
          <a href="#about" className={styles.navLink}>关于</a>
          <ThemeToggle theme={theme} onToggle={toggle} />
        </div>
      </nav>

      <div className={styles.content}>
        {/* ── Section 1: Hero ── */}
        <section className={styles.hero} id="hero" aria-labelledby="hero-title">
          <div className={styles.heroBadge} role="text">
            <span className={styles.heroBadgeDot} aria-hidden="true" />
            穹沛科技 · 自主导航系统 v1.7.5
          </div>

          <h1 className={styles.heroTitle} id="hero-title">
            让机器人走进<br />
            <span className={styles.heroTitleAccent}>真实世界</span>
          </h1>

          <p className={styles.heroSub}>
            全栈自研导航系统 — 从电机到云端<br />
            跑在 S100P 四足机器人上的生产级软件
          </p>

          <div className={styles.heroCta}>
            <a href="#highlights" className={styles.btnPrimary} aria-label="了解更多产品信息">
              了解更多
            </a>
            <a href="#architecture" className={styles.btnSecondary} aria-label="查看系统架构">
              查看架构
            </a>
          </div>

          <img
            src="/s100p.png"
            className={styles.heroVisual}
            alt="地瓜机器人 RDK S100P — 128 TOPS Nash BPU"
            draggable="false"
          />

          <div className={styles.heroSweep} aria-hidden="true" />
        </section>

        {/* ── Section 2: Highlights ── */}
        <section className={styles.section} id="highlights" aria-labelledby="highlights-title">
          <div className={`${styles.reveal}`}>
            <span className={styles.sectionLabel}>产品亮点</span>
            <h2 className={styles.sectionTitle} id="highlights-title">
              为真实环境而生
            </h2>
            <p className={styles.sectionBody}>
              从地下室的 CAN 总线到山坡上的 SLAM 点云，每一个模块都在实际硬件上验证过。
            </p>
          </div>

          <div className={styles.highlightsGrid} role="list">
            {HIGHLIGHTS.map((h, i) => (
              <div
                key={h.title}
                className={`${styles.card} ${styles.reveal}`}
                data-delay={String(i * 90)}
                role="listitem"
              >
                <div className={styles.cardIcon} aria-hidden="true">{h.icon}</div>
                <h3 className={styles.cardTitle}>{h.title}</h3>
                <p className={styles.cardDesc}>{h.desc}</p>
              </div>
            ))}
          </div>
        </section>

        {/* ── Section 3: Architecture ── */}
        <div className={styles.archSection} id="architecture">
          <div className={styles.archInner}>
            <div className={`${styles.archHeader} ${styles.reveal}`}>
              <span className={styles.sectionLabel}>系统架构</span>
              <h2 className={styles.sectionTitle} id="arch-title">
                7 层模块化设计
              </h2>
              <p className={styles.sectionBody} style={{ margin: '0 auto' }}>
                Product 驱动架构 — 现场进程所有权明确，Host 内组件保持单向依赖并可独立替换。
              </p>
            </div>

            <div className={styles.layers} role="list" aria-label="系统架构层次">
              {LAYERS.map((layer, i) => (
                <div
                  key={layer.l}
                  className={styles.layer}
                  data-l={String(layer.l)}
                  ref={(el) => { layerRefs.current[i] = el }}
                  role="listitem"
                  style={{ transitionDelay: `${i * 60}ms` }}
                >
                  <span className={styles.layerNum} aria-label={`第 ${layer.l} 层`}>
                    L{layer.l}
                  </span>
                  <div className={styles.layerBody}>
                    <span className={styles.layerName}>{layer.name}</span>
                    <span className={styles.layerDesc}>{layer.desc}</span>
                  </div>
                  <span className={styles.layerTag} aria-hidden="true">
                    {layer.modules.split(' · ')[0]}
                  </span>
                </div>
              ))}
            </div>
          </div>
        </div>

        {/* ── Section 4: Stats ── */}
        <section className={styles.statsSection} id="stats" aria-labelledby="stats-title">
          <div className={`${styles.reveal}`}>
            <span className={styles.sectionLabel}>性能数据</span>
            <h2 className={styles.sectionTitle} id="stats-title">
              实测，不是宣传
            </h2>
          </div>

          <div className={styles.statsGrid} role="list">
            {STATS.map((s, i) => (
              <div
                key={s.label}
                className={`${styles.statCell} ${styles.reveal}`}
                data-delay={String(i * 100)}
                role="listitem"
              >
                <div className={styles.statValue} aria-label={`${s.end}${s.suffix}`}>
                  <CountUp end={s.end} suffix={s.suffix} duration={1600} />
                </div>
                <div className={styles.statLabel}>{s.label}</div>
                <div className={styles.statNote}>{s.note}</div>
              </div>
            ))}
          </div>
        </section>

        {/* ── Section 5: About ── */}
        <div className={styles.aboutSection} id="about">
          <div className={styles.aboutInner}>
            <div className={`${styles.reveal}`}>
              <h2 className={styles.aboutTitle}>
                穹沛科技<br />做机器人的人
              </h2>
              <p className={styles.aboutBody}>
                我们是一支全栈机器人团队。从 FOC 电机固件到 RL 运动策略，
                从 SLAM 建图到云端 OTA，每一层都自主研发、自主验证。
                LingTu 是我们最接近商业化的产品——跑在真实四足机器人上的自主导航系统。
              </p>
              <div className={styles.aboutLinks}>
                <a
                  href="https://github.com/Kitjesen"
                  className={styles.aboutLink}
                  target="_blank"
                  rel="noopener noreferrer"
                  aria-label="访问 GitHub 主页"
                >
                  <svg width="15" height="15" viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
                    <path d="M12 2C6.477 2 2 6.484 2 12.017c0 4.425 2.865 8.18 6.839 9.504.5.092.682-.217.682-.483 0-.237-.008-.868-.013-1.703-2.782.605-3.369-1.343-3.369-1.343-.454-1.158-1.11-1.466-1.11-1.466-.908-.62.069-.608.069-.608 1.003.07 1.531 1.032 1.531 1.032.892 1.53 2.341 1.088 2.91.832.092-.647.35-1.088.636-1.338-2.22-.253-4.555-1.113-4.555-4.951 0-1.093.39-1.988 1.029-2.688-.103-.253-.446-1.272.098-2.65 0 0 .84-.27 2.75 1.026A9.564 9.564 0 0 1 12 6.844a9.59 9.59 0 0 1 2.504.337c1.909-1.296 2.747-1.027 2.747-1.027.546 1.379.202 2.398.1 2.651.64.7 1.028 1.595 1.028 2.688 0 3.848-2.339 4.695-4.566 4.943.359.309.678.92.678 1.855 0 1.338-.012 2.419-.012 2.747 0 .268.18.58.688.482A10.02 10.02 0 0 0 22 12.017C22 6.484 17.522 2 12 2z" />
                  </svg>
                  GitHub
                </a>
                <a
                  href="mailto:contact@inovxio.com"
                  className={styles.aboutLink}
                  aria-label="发送邮件联系我们"
                >
                  <svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
                    <rect x="2" y="4" width="20" height="16" rx="2"/>
                    <path d="m22 7-8.97 5.7a1.94 1.94 0 0 1-2.06 0L2 7"/>
                  </svg>
                  联系我们
                </a>
                <a
                  href="?dashboard"
                  className={styles.aboutLink}
                  aria-label="打开控制台"
                >
                  <svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
                    <rect x="3" y="3" width="7" height="7" rx="1"/>
                    <rect x="14" y="3" width="7" height="7" rx="1"/>
                    <rect x="3" y="14" width="7" height="7" rx="1"/>
                    <rect x="14" y="14" width="7" height="7" rx="1"/>
                  </svg>
                  打开控制台
                </a>
              </div>
            </div>
          </div>
        </div>

        {/* ── Footer ── */}
        <footer className={styles.footer} role="contentinfo">
          <a href="#" className={styles.footerLogo} aria-label="LingTu 首页">
            <div className={styles.footerMark} aria-hidden="true" />
            灵途 LingTu
          </a>
          <span className={styles.footerCopy}>
            © 2025 上海穹沛科技有限公司
          </span>
          <nav className={styles.footerLinks} aria-label="页脚链接">
            <a href="#hero" className={styles.footerLink}>首页</a>
            <a href="#highlights" className={styles.footerLink}>产品</a>
            <a href="#architecture" className={styles.footerLink}>架构</a>
            <a href="#about" className={styles.footerLink}>关于</a>
          </nav>
        </footer>
      </div>
    </div>
  )
}
