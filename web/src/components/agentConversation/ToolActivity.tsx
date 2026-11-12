import { useEffect, useRef, useState } from 'react'
import { Roll } from '../stageshow/motion/Roll'
import { Collapse } from '../stageshow/motion/Collapse'
import { Check, ChevronDown, CircleAlert, Copy, Image, LoaderCircle, Maximize2, Pencil, Search, Terminal, Wrench, X } from 'lucide-react'
import { ModalShell } from '../Modal'
import { SavedMapsResult } from './SavedMapsResult'
import { SavedMapCard } from './SavedMapCard'
import { ObservationCard, type ObservationView } from './ObservationCard'
import { LiveCameraCard } from './LiveCameraCard'
import { CloudPreview } from './CloudPreview'
import { SensorResult } from './SensorResult'
import { conversationToolNames as toolNames, type ConversationMessage, type ConversationOptions } from '../../services/agentConversation.ts'
import styles from '../AgentConversation.module.css'

function ActivityIcon({ name }: { name: string }) {
  const Icon = /exec|shell|command|python/.test(name) ? Terminal
    : /save|edit|set_navigation/.test(name) ? Pencil
      : /image|camera|detect/.test(name) ? Image
        : /search|query|find|inspect|list/.test(name) ? Search : Wrench
  return <Icon size={14} aria-hidden="true" />
}

function WebSources({ sources, extracted }: { sources: unknown[]; extracted: boolean }) {
  const entries = sources.flatMap(item => {
    if (!item || typeof item !== 'object') return []
    const source = item as Record<string, unknown>
    if (typeof source.url !== 'string' || !/^https?:\/\//i.test(source.url)) return []
    return [{ url: source.url, title: typeof source.title === 'string' ? source.title : source.url,
      content: typeof source.content === 'string' ? source.content : '', truncated: source.truncated === true }]
  })
  if (!entries.length) return <p className={styles.toolReason}>没有找到可展示的来源。</p>
  return <section className={styles.webSources} aria-label={extracted ? '网页正文' : '搜索来源'}>
    <header>{extracted ? '网页正文' : `${entries.length} 个来源`}<span>Tavily · {extracted ? '提取结果' : '搜索摘要'}</span></header>
    {entries.map((source, index) => <article key={`${source.url}-${index}`}>
      <a href={source.url} target="_blank" rel="noreferrer"><span>{index + 1}</span><strong>{source.title}</strong><span aria-hidden="true">↗</span></a>
      <small className={styles.webSourceUrl}>{source.url}</small>
      {extracted ? <details><summary>展开正文</summary><p>{source.content}</p></details> : <p>{source.content}</p>}
      {source.truncated && <small>内容已截断{extracted ? '，完整内容请打开原网页' : '，可让助手继续读取网页'}。</small>}
    </article>)}
  </section>
}

function ToolDetails({ text, result }: { text: string; result: Record<string, unknown> }) {
  const command = typeof result.command === 'string' ? result.command : typeof result.cmd === 'string' ? result.cmd : undefined
  const output = typeof result.stdout === 'string' ? result.stdout : typeof result.output === 'string' ? result.output : undefined
  const stderr = typeof result.stderr === 'string' ? result.stderr : undefined
  const structured = command != null || output != null || stderr != null
  const raw = Object.keys(result).length ? JSON.stringify(result, null, 2) : text
  return <div className={styles.toolDetailContent}>
    {command != null && <section><h4>命令</h4><pre>{command || '（空命令）'}</pre></section>}
    {typeof result.cwd === 'string' && <p className={styles.toolDirectory}>目录 <code>{result.cwd}</code></p>}
    {output != null && <section><h4>输出</h4><pre>{output || '（无输出）'}</pre></section>}
    {stderr != null && stderr !== '' && <section><h4>标准错误</h4><pre>{stderr}</pre></section>}
    {structured ? <details className={styles.rawToolResult}><summary>原始结果</summary><pre>{raw}</pre></details> : <pre>{raw}</pre>}
  </div>
}

export function ToolActivityGroup({ messages, active, runningId, awaiting, now, onSelectModel }: {
  messages: ConversationMessage[]; active: boolean; runningId?: string; awaiting: string[]; now?: number; onSelectModel?: (model: string) => void
}) {
  messages = messages.filter((message, index) => {
    if (message.kind !== 'tool_start') return true
    try { return messages[index + 1]?.kind !== JSON.parse(message.text).tool }
    catch { return true }
  })
  const names = new Set<string>()
  let keepVisible = active || messages.length <= 3
  for (const message of messages) {
    try {
      const result = JSON.parse(message.text)
      names.add(message.metadata?.display_name ?? (message.kind === 'tool_start' ? String(result?.tool ?? '工具') : message.kind ?? '工具'))
      keepVisible ||= message.kind === 'web_search' || message.kind === 'web_extract'
      // Errors and map-preview actions remain visible without opening the history.
      keepVisible ||= message.metadata?.status === 'failed' || Boolean(result?.observation_view || result?.camera_view || result?.map_view || result?.error || result?.preview_request || result?.sensor || result?.sensors || message.kind === 'get_model_connections') || result?.success === false || result?.ok === false || result?.accepted === false || (typeof result?.exit_code === 'number' && result.exit_code !== 0)
    } catch { names.add(message.kind ?? '工具') }
  }
  const rows = <ol className={styles.toolSteps} aria-label="执行步骤">{messages.map(message => <li key={message.id} className={styles.toolStep}>
    <ToolActivity message={message} onSelectModel={onSelectModel} now={now} running={message.id === runningId} awaiting={awaiting} />
  </li>)}</ol>
  if (keepVisible) return rows
  return <details className={styles.toolGroup} onKeyDown={event => {
    if (event.key === 'Escape' && event.currentTarget.open) {
      event.preventDefault(); event.stopPropagation(); event.currentTarget.open = false
      event.currentTarget.querySelector('summary')?.focus()
    }
  }}>
    <summary aria-label="展开工具过程"><ActivityIcon name={[...names][0] ?? ''} /><span>{[...names].map(name => toolNames[name] ?? name).join(' · ')}</span>
      <ChevronDown size={12} /></summary>
    {rows}
  </details>
}

export function ToolActivity({ message, running = false, awaiting = [], now, onSelectModel }: { message: ConversationMessage; running?: boolean; awaiting?: string[]; now?: number; onSelectModel?: (model: string) => void }) {
  const [expanded, setExpanded] = useState(false)
  const [enlarged, setEnlarged] = useState(false)
  const [copyState, setCopyState] = useState('')
  const toggle = useRef<HTMLButtonElement>(null)
  useEffect(() => {
    if (copyState !== 'done') return
    const timer = setTimeout(() => setCopyState(''), 1800)
    return () => clearTimeout(timer)
  }, [copyState])
  let result: Record<string, unknown> = {}
  try { const parsed = JSON.parse(message.text); if (parsed && typeof parsed === 'object' && !Array.isArray(parsed)) result = parsed } catch { /* Older tools returned plain text. */ }
  const started = message.metadata?.status === 'running' || message.kind === 'tool_start'
  const name = message.kind === 'tool_start' ? String(result.tool ?? '工具') : message.kind ?? '工具'
  const displayName = message.metadata?.display_name ?? (name === 'specialist_review' && typeof result.specialist === 'string' ? result.specialist : toolNames[name] ?? name)
  const exitCode = typeof result.exit_code === 'number' ? result.exit_code : undefined
  const failed = message.metadata?.status === 'failed' || Boolean(result.error) || result.success === false || result.ok === false || result.accepted === false || (exitCode != null && exitCode !== 0)
  const reason = result.error ?? result.reason
  const waiting = started && awaiting.includes(name)
  const elapsed = message.metadata?.duration_ms ?? (started && now != null && message.metadata?.started_at != null ? Math.max(0, (now - message.metadata.started_at) * 1000) : undefined)
  const state = message.metadata?.status === 'cancelled' ? 'cancelled' : failed ? 'failed' : started ? waiting ? 'waiting' : running || name === 'specialist_review' ? 'running' : 'pending' : 'completed'
  const status = { cancelled: '已中断', failed: '未完成', waiting: '待确认', running: '调用中', pending: '已发起', completed: '已返回' }[state]
  const recipe = Boolean(result.preview_request)
  const observationView = name === 'view_robot_card' && ['robot', 'joints', 'imu', 'navigation'].includes(String(result.observation_view)) ? result.observation_view as ObservationView : null
  const cameraView = name === 'view_live_camera' && (result.camera_view as { source?: unknown } | undefined)?.source === 'robot'
  const view = result.map_view as { map_id?: unknown; content_epoch?: unknown } | undefined
  const mapView = name === 'view_saved_map' && view && typeof view.map_id === 'string' && view.map_id.trim()
    && Number.isSafeInteger(view.content_epoch) && Number(view.content_epoch) > 0
    ? { name: view.map_id, epoch: Number(view.content_epoch) } : null
  const hasResultCard = !started && !failed && (result.sensor != null || result.sensors != null
    || observationView || cameraView || mapView || name === 'list_saved_maps' && Array.isArray(result.maps)
    || (name === 'web_search' || name === 'web_extract') && Array.isArray(result.sources))

  const connections = new Map<string, ConversationOptions['models']>()
  if (name === 'get_model_connections' && Array.isArray(result.models)) {
    for (const model of result.models as ConversationOptions['models']) {
      const provider = model.provider ?? model.id
      connections.set(provider, [...(connections.get(provider) ?? []), model])
    }
  }

  const copyOutput = async () => {
    try { await navigator.clipboard.writeText(message.text); setCopyState('done') }
    catch { setCopyState('failed') }
  }
  return <div className={styles.toolActivity} data-state={state} data-failed={failed} data-running={state === 'running'}
    onKeyDown={event => {
      if (event.key === 'Escape' && expanded && !enlarged) {
        event.preventDefault(); event.stopPropagation(); setExpanded(false); toggle.current?.focus()
      }
    }}>
    <button ref={toggle} type="button" className={styles.toolToggle} aria-expanded={expanded} onClick={() => setExpanded(!expanded)}>
      <span className={styles.stepMark} aria-hidden="true">
        {failed ? <CircleAlert size={14} /> : state === 'running' ? <LoaderCircle size={14} className={styles.toolSpinner} />
          : <svg width={14} height={14} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2}
            strokeLinecap="round" strokeLinejoin="round"><path d="m5 12 4 4L19 6" pathLength={1} /></svg>}
      </span>
      <strong>{displayName}</strong><Roll align="start">{status}</Roll>{exitCode != null && <small className={styles.toolDuration}>退出 {exitCode}</small>}{elapsed != null && <small className={styles.toolDuration} title="单次工具调用时间，不含模型生成；包含等待确认">{elapsed < 1000 ? `${Math.round(elapsed)} ms` : `${(elapsed / 1000).toFixed(1)} 秒`}</small>}<ChevronDown size={12} className={styles.toolChevron} />
    </button>
    {name === 'specialist_review' && <div className={styles.specialistResult}>
      {typeof result.task === 'string' && <details><summary>检查任务</summary><p>{result.task}</p></details>}
      {typeof result.summary === 'string' && result.summary && <p>{result.summary}</p>}
      {Array.isArray(result.evidence) && result.evidence.length > 0 && <small>{result.evidence.length} 条工具回执 · 展开详情查看</small>}
    </div>}
    {hasResultCard && <div className={styles.resultArrival}><div>
    {observationView && <ObservationCard view={observationView} />}
    {cameraView && <LiveCameraCard />}
    {mapView && <SavedMapCard key={`${mapView.name}:${mapView.epoch}`} name={mapView.name} epoch={mapView.epoch} />}
    {!started && !failed && (result.sensor != null || result.sensors != null) && <SensorResult result={result} />}
    {name === 'list_saved_maps' && !started && !failed && Array.isArray(result.maps) && <SavedMapsResult maps={result.maps} />}
    {(name === 'web_search' || name === 'web_extract') && !started && !failed && Array.isArray(result.sources)
      && <WebSources sources={result.sources} extracted={name === 'web_extract'} />}
    </div></div>}
    {connections.size > 0 && <div className={styles.connectionCards}>
      {[...connections].map(([provider, models]) => {
        const ready = models.filter(model => model.configured)
        const label = ({ moonshot: 'Kimi', openai: 'OpenAI', codex: 'ChatGPT / Codex', claude: 'Claude', qwen: 'Qwen' } as Record<string, string>)[provider] ?? models[0].label
        return <section key={provider} className={styles.connectionCard}>
          <span className={styles.connectionLogo} aria-hidden="true">{label.slice(0, 1)}</span>
          <div><strong>{label}</strong><small>{models.length} 个模型 · {ready.length ? provider === 'codex' ? '已授权' : '已配置' : provider === 'codex' ? '请在模型连接中登录' : `未配置 · ${models[0].key_env ?? '需要服务端配置'}`}</small></div>
          {ready.length > 0 && onSelectModel && <details className={styles.connectionModels}>
            <summary>选择模型</summary><div>{ready.map(model => <button key={model.id} type="button" onClick={event => {
              onSelectModel(model.id)
              event.currentTarget.closest('details')?.removeAttribute('open')
            }}>{model.label}</button>)}</div>
          </details>}
        </section>
      })}
    </div>}
    {failed && typeof reason === 'string' && <p className={styles.toolReason}>{reason}</p>}
    {!recipe && !result.sensor && typeof result.point_count === 'number' && <p>{result.point_count.toLocaleString()} 点
      {typeof result.removed_points === 'number' && ` · 移除 ${result.removed_points.toLocaleString()}`}
      {typeof result.added_points === 'number' && ` · 新增 ${result.added_points.toLocaleString()}`}</p>}
    {recipe && !started && !failed && <CloudPreview result={result} />}
    <Collapse open={expanded} inert={!expanded} aria-hidden={!expanded} className={styles.toolDisclosure}>
    {expanded && <section className={styles.toolDetails} aria-label="执行详情">
      <header className={styles.toolDetailToolbar}><Terminal size={13} /><span>执行详情</span>
        <button type="button" onClick={() => void copyOutput()} aria-label="复制执行结果" title="复制执行结果">{copyState === 'done' ? <Check size={13} /> : <Copy size={13} />}</button>
        <button type="button" onClick={() => setEnlarged(true)} aria-label="放大执行详情" title="放大执行详情"><Maximize2 size={13} /></button>
      </header>
      <ToolDetails text={message.text} result={result} />
      {copyState && <span role="status" className={copyState === 'failed' ? styles.toolReason : styles.screenReaderOnly}>{copyState === 'failed' ? '复制失败，请选中文字复制' : '已复制'}</span>}
    </section>}
    </Collapse>
    {enlarged && <ModalShell open title={displayName} className={styles.toolDetailDialog} onClose={() => setEnlarged(false)}
      headerAction={<div className={styles.toolDialogActions}>
        <button type="button" onClick={() => void copyOutput()} aria-label="复制执行结果" title="复制执行结果">{copyState === 'done' ? <Check size={15} /> : <Copy size={15} />}</button>
        <button type="button" data-modal-cancel onClick={() => setEnlarged(false)} aria-label="收起执行详情" title="收起执行详情"><X size={17} /></button>
      </div>}>
      <div className={styles.toolDetailSummary}><span>{status}</span>{exitCode != null && <span>退出码 {exitCode}</span>}{elapsed != null && <span>{elapsed < 1000 ? `${Math.round(elapsed)} ms` : `${(elapsed / 1000).toFixed(1)} 秒`}</span>}</div>
      <ToolDetails text={message.text} result={result} />
      {copyState && <span role="status" className={copyState === 'failed' ? styles.toolReason : styles.screenReaderOnly}>{copyState === 'failed' ? '复制失败，请选中文字复制' : '已复制'}</span>}
    </ModalShell>}
  </div>
}
