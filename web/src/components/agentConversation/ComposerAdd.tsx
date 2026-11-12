import { useRef } from 'react'
import { Activity, Camera, ClipboardList, Cpu, Radar, CircleStop, Compass, FileUp, History, Map, Plus, ScanSearch, Settings2, ShieldCheck } from 'lucide-react'
import type { Locale } from '../../i18n'
import styles from '../AgentConversation.module.css'
import { useImuMonitor } from '../../hooks/useImuMonitor.ts'
import { useSensorMonitor } from '../../hooks/useSensorMonitor.ts'

export function ComposerAdd({ locale, disabled, hasMap, onFiles, onPrompt }: {
  locale: Locale; disabled: boolean; hasMap: boolean; onFiles: () => void; onPrompt: (prompt: string) => void
}) {
  const menu = useRef<HTMLDetailsElement>(null)
  const monitor = useImuMonitor()
  const sensors = useSensorMonitor()
  const text = (en: string, zh: string) => locale === 'zh' ? zh : en
  return <details ref={menu} className={`${styles.selectMenu} ${styles.addMenu}`} name="lingtu-menu"
    onToggle={event => {
      if (!event.currentTarget.open) return
      const element = event.currentTarget
      const composer = element.closest('form')!
      element.style.setProperty('--add-space', `${Math.max(0, composer.getBoundingClientRect().top - 16)}px`)
    }}
    onBlur={event => { if (event.relatedTarget && !event.currentTarget.contains(event.relatedTarget)) event.currentTarget.open = false }}
    onKeyDown={event => {
      if (disabled) return
      const element = event.currentTarget
      if (event.key === 'Escape') {
        event.preventDefault(); event.stopPropagation(); element.open = false
        element.querySelector('summary')?.focus()
      } else if (['ArrowDown', 'ArrowUp', 'Home', 'End'].includes(event.key)) {
        event.preventDefault(); element.open = true
        const items = [...element.querySelectorAll<HTMLButtonElement>('[role="menuitem"]')]
        const index = items.indexOf(document.activeElement as HTMLButtonElement)
        const next = event.key === 'Home' ? 0 : event.key === 'End' ? items.length - 1 : index < 0 ? event.key === 'ArrowUp' ? items.length - 1 : 0
          : (index + (event.key === 'ArrowDown' ? 1 : -1) + items.length) % items.length
        items[next]?.focus()
      }
    }}>
    <summary className={styles.iconButton} aria-label={text('Add content', '添加内容')} aria-haspopup="menu"
      aria-disabled={disabled} tabIndex={disabled ? -1 : 0} onClick={event => { if (disabled) event.preventDefault() }}>
      <Plus size={19} aria-hidden="true" />
    </summary>
    <div className={`${styles.selectPanel} ${styles.addPanel}`} role="menu" aria-label={text('Robot tools and plugins', '机器人功能与插件')}>
      <div className={styles.menuHeading}>{text('Add', '添加')}</div>
      <div className={styles.addSectionLabel}>{text('Robot', '机器人')}</div>
      <button type="button" role="menuitem" disabled={disabled} onClick={() => {
        if (menu.current) menu.current.open = false
        menu.current?.querySelector('summary')?.focus()
        onPrompt(text('Prepare a draft plan for the task I describe next. Do not execute actions; present the plan first.', '请为我接下来描述的任务制定方案，先给出方案草稿，不执行动作。'))
      }} data-tone="map"><ClipboardList className={styles.addIcon} size={15} aria-hidden="true" /><span><strong>{text('Draft a plan', '制定方案')}</strong><small>{text('Review the steps before execution', '先整理步骤，再决定如何执行')}</small></span></button>
      <button type="button" role="menuitem" disabled={disabled} onClick={() => {
        if (menu.current) menu.current.open = false
        menu.current?.querySelector('summary')?.focus()
        onPrompt(text('Check the robot status and current task.', '显示机器人状态实时卡片。'))
      }} data-tone="status"><Activity className={styles.addIcon} size={15} aria-hidden="true" /><span><strong>{text('Robot status', '机器人状态')}</strong><small>{text('Position, task and connection', '位置、任务与连接')}</small></span></button>
      {monitor && <button type="button" role="menuitem" disabled={disabled} data-tone="status" onClick={() => {
        if (menu.current) menu.current.open = false
        monitor()
      }}><Activity className={styles.addIcon} size={15} aria-hidden="true" /><span><strong>IMU 实时监测</strong><small>曲线、记录与数据导出</small></span></button>}
      {sensors && <button type="button" role="menuitem" disabled={disabled} data-tone="status" onClick={() => {
        if (menu.current) menu.current.open = false
        sensors('catalog', menu.current?.querySelector('summary') ?? undefined)
      }}><Cpu className={styles.addIcon} size={15} aria-hidden="true" /><span><strong>{text('Sensor cards', '传感器组件')}</strong><small>{text('Camera, LiDAR, joints and measurements', '相机、雷达、关节与测量')}</small></span></button>}
      {[
        { Icon: Cpu, title: '关节与故障', hint: '实时值与错误状态', prompt: '显示关节与故障实时卡片。' },
        { Icon: Activity, title: 'IMU 与记录', hint: '曲线与记录回看', prompt: '显示 IMU 与记录卡片，不自动开始录制。' },
        { Icon: Compass, title: '导航任务', hint: '当前阶段与停止原因', prompt: '显示导航任务实时卡片，不执行运动。' },
        { Icon: Cpu, title: text('Robot sensors', '机器人传感器'), hint: text('IMU, joints and motors', 'IMU、关节与电机'), prompt: text('Discover the current robot capabilities, then read available IMU, joint and motor observations. Report missing sources.', '先查询当前机器人的观测能力，再读取可用的 IMU、关节和电机信息；说明未接入的数据。') },
        { Icon: Camera, title: text('Live camera', '实时相机'), hint: text('Expand to watch', '展开卡片观看'), prompt: text('Show the live robot camera card for viewing only. Do not record or move the robot.', '显示机器人实时相机卡片，仅观看，不录制、不控制运动。') },
        { Icon: Camera, title: text('Camera snapshot', '相机快照'), hint: text('Capture and inspect an image', '获取图片并查看'), prompt: text('Capture the robot camera and describe what is visible. Report if the image is cached or unavailable.', '获取机器人相机快照并描述画面；说明图片的来源与可用状态。') },
        { Icon: Radar, title: text('LiDAR and navigation', '雷达与导航'), hint: text('Scan and native stop reasons', '扫描与原生停车原因'), prompt: text('Inspect the current LiDAR scan and native navigation diagnostics. Explain obstacles and stop reasons without inferring physical contact.', '查看当前 LiDAR 扫描和原生导航诊断，解释障碍及停车原因，不把点云直接当作实体接触证据。') },
      ].map(({ Icon, title, hint, prompt }) => <button key={title} type="button" role="menuitem" disabled={disabled} data-tone="status" onClick={() => {
        if (menu.current) menu.current.open = false
        menu.current?.querySelector('summary')?.focus()
        onPrompt(prompt)
      }}><Icon className={styles.addIcon} size={15} aria-hidden="true" /><span><strong>{title}</strong><small>{hint}</small></span></button>)}
      <button type="button" role="menuitem" disabled={disabled} onClick={() => {
        if (menu.current) menu.current.open = false
        menu.current?.querySelector('summary')?.focus()
        onPrompt(text('Show the available saved maps.', '查看可用的已保存地图。'))
      }} data-tone="map"><Map className={styles.addIcon} size={15} aria-hidden="true" /><span><strong>{text('Saved maps', '已保存地图')}</strong><small>{text('Inspect map readiness', '检查地图是否可用')}</small></span></button>
      <button type="button" role="menuitem" disabled={disabled} onClick={() => {
        if (menu.current) menu.current.open = false
        menu.current?.querySelector('summary')?.focus()
        onPrompt(text('Navigate to a target I describe next.', '准备前往我接下来描述的目标。'))
      }} data-tone="navigate"><Compass className={styles.addIcon} size={15} aria-hidden="true" /><span><strong>{text('Navigate to a target', '前往目标')}</strong><small>{text('Describe the target in the chat', '在对话中描述目标')}</small></span></button>
      <button type="button" role="menuitem" disabled={disabled} onClick={() => {
        if (menu.current) menu.current.open = false
        menu.current?.querySelector('summary')?.focus()
        onPrompt(text('Stop the current robot task.', '停止当前机器人任务。'))
      }} data-tone="stop"><CircleStop className={styles.addIcon} size={15} aria-hidden="true" /><span><strong>{text('Stop current task', '停止当前任务')}</strong><small>{text('Add a stop request to the chat', '添加停止请求到输入框')}</small></span></button>
      {hasMap && <button type="button" role="menuitem" disabled={disabled} onClick={() => {
        if (menu.current) menu.current.open = false
        menu.current?.querySelector('summary')?.focus()
        onPrompt(text('Inspect this map and its navigation regions.', '检查当前地图和通行区域。'))
      }} data-tone="map"><ScanSearch className={styles.addIcon} size={15} aria-hidden="true" /><span><strong>{text('Inspect current map', '检查当前地图')}</strong><small>{text('Read map and no-go regions', '查看地图与禁行区域')}</small></span></button>}
      {hasMap && <button type="button" role="menuitem" disabled={disabled} onClick={() => {
        if (menu.current) menu.current.open = false
        menu.current?.querySelector('summary')?.focus()
        onPrompt(text('Help me preview or set a no-go region on the current map.', '帮我预览或设置当前地图的禁行区域。'))
      }} data-tone="edit"><Settings2 className={styles.addIcon} size={15} aria-hidden="true" /><span><strong>{text('Map editing', '地图编辑')}</strong><small>{text('Preview edits or set a no-go region', '预览编辑或设置禁行区域')}</small></span></button>}
      <div className={styles.addSectionLabel}>{text('Plugins and components', '插件与组件')}</div>
      <button type="button" role="menuitem" disabled={disabled} onClick={() => {
        if (menu.current) menu.current.open = false
        menu.current?.querySelector('summary')?.focus()
        onFiles()
      }} data-tone="files"><FileUp className={styles.addIcon} size={15} aria-hidden="true" /><span><strong>{text('Files and images', '文件与图片')}</strong><small>{text('Attach context to this chat', '给当前对话添加上下文')}</small></span></button>

      <button type="button" role="menuitem" disabled={disabled} onClick={() => {
        if (menu.current) menu.current.open = false
        menu.current?.querySelector('summary')?.focus()
        onPrompt(text('Show recent robot tasks and conversation history.', '查看最近的机器人任务和对话历史。'))
      }} data-tone="history"><History className={styles.addIcon} size={15} aria-hidden="true" /><span><strong>{text('Recent tasks', '最近任务')}</strong><small>{text('Continue from the current conversation', '从当前对话继续')}</small></span></button>
      <div className={styles.addSectionLabel}>{text('Safety', '安全')}</div>
      <button type="button" role="menuitem" disabled={disabled} onClick={() => {
        if (menu.current) menu.current.open = false
        menu.current?.querySelector('summary')?.focus()
        onPrompt(text('Explain the current safety and control state.', '说明当前安全和控制状态。'))
      }} data-tone="safety"><ShieldCheck className={styles.addIcon} size={15} aria-hidden="true" /><span><strong>{text('Safety state', '安全状态')}</strong><small>{text('Control mode and stop state', '控制模式与急停状态')}</small></span></button>

    </div>
  </details>
}
