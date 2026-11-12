import { memo, useEffect, useLayoutEffect, useRef, useState } from 'react'
import { QueuedMessages } from './agentConversation/QueuedMessages'
import { Archive, ArchiveRestore, ArrowUp, ArrowUpRight, BrainCircuit, Check, ChevronDown, Copy, Ellipsis, GitBranch, History, Map as MapIcon, MessageCircle, Paperclip, Pencil, Plus, RotateCw, Scan, Search, Share2, ShieldCheck, SlidersHorizontal, X, Zap, CornerUpLeft, Maximize2, Minimize2, Download, FileText, Users } from 'lucide-react'
import type { Locale } from '../i18n'
import type { useAgentConversation } from '../hooks/useAgentConversation'
import { renderMarkdown } from '../guide/markdown'
import { useMenuDismiss } from './menuEvents'
import { EffortGlyph } from './agentConversation/EffortGlyph'
import { ComposerAdd } from './agentConversation/ComposerAdd.tsx'
import { ConversationShareCard } from './ConversationShareCard'
import { useMapSettings } from '../hooks/useMapSettings.ts'
import { QuestionCard } from './agentConversation/QuestionCard.tsx'
import { PlanCard } from './agentConversation/PlanCard.tsx'
import { ToolActivityGroup } from './agentConversation/ToolActivity.tsx'
import { Bot, type BotState } from './agentConversation/Bot.tsx'
import { BotStudio } from './agentConversation/BotStudio'
import { ModelConnections } from './agentConversation/ModelConnections'
import { readBotProfile, saveBotProfile } from '../services/botProfile.ts'
import { addModelMarks } from './agentConversation/modelMentions.ts'
import { replyMarkdownBlocks } from './agentConversation/markdownStreamBlocks.ts'
import styles from './AgentConversation.module.css'
import { useImuMonitor } from '../hooks/useImuMonitor.ts'
import { useSensorMonitor } from '../hooks/useSensorMonitor.ts'
import { sensorCards } from '../services/sensors.ts'

import { conversationAttachmentUrl, conversationToolNames as toolNames, type ConversationMessage, type ConversationTurn } from '../services/agentConversation.ts'

const EMPTY_MESSAGES: ConversationMessage[] = []

interface AgentConversationProps {
  locale: Locale
  controller: ReturnType<typeof useAgentConversation>
}

function InlineBusinessCard({ card }: { card: NonNullable<NonNullable<ConversationMessage['metadata']>['card']> }) {
  const actions = card.actions?.filter(action => action && typeof action.label === 'string' && typeof action.href === 'string') ?? []
  return <section className={styles.inlineCard} aria-label={card.title ?? '业务卡片'}>
    {card.image_url && <img src={card.image_url} alt="" loading="lazy" />}
    <div className={styles.inlineCardBody}>
      {card.title && <strong>{card.title}</strong>}
      {card.subtitle && <span>{card.subtitle}</span>}
      {card.price && <b>{card.price}</b>}
      {card.detail && <small>{card.detail}</small>}
      {actions.length > 0 && <div className={styles.inlineCardActions}>{actions.map(action => <a key={`${action.label}-${action.href}`} href={action.href} className={styles.cardAction}>{action.label}</a>)}</div>}
    </div>
  </section>
}

function CollaborationCard({ collaboration }: { collaboration: NonNullable<NonNullable<ConversationMessage['metadata']>['collaboration']> }) {
  const content = <><Users size={14} aria-hidden="true" /><span><strong>{collaboration.name}</strong>{collaboration.status && <small>{collaboration.status}</small>}{collaboration.summary && <em>{collaboration.summary}</em>}</span></>
  return collaboration.href ? <a className={styles.collaborationCard} href={collaboration.href}>{content}</a> : <div className={styles.collaborationCard}>{content}</div>
}

function MessageReactions() {
  const [selected, setSelected] = useState<string | null>(null)
  const reactions = ['👍', '✅', '❤️']
  return <div className={styles.messageReactions} aria-label="消息回应">
    {reactions.map(reaction => <button key={reaction} type="button" aria-pressed={selected === reaction}
      onClick={() => setSelected(current => current === reaction ? null : reaction)}>{reaction}</button>)}
  </div>
}

function TurnNotice({ turn, locale }: { turn: ConversationTurn; locale: Locale }) {
  const fallback = turn.state === 'interrupted' ? ['This turn was interrupted.', '本轮对话已中断。']
    : turn.state === 'cancelled' ? ['This turn was cancelled.', '本轮对话已取消。']
      : ['This turn failed.', '本轮对话未完成。']
  return <p className={styles.turnNotice} data-state={turn.state} role="status">
    {turn.text || turn.failure_reason || fallback[locale === 'zh' ? 1 : 0]}
  </p>
}

const ReplyBlock = memo(function ReplyBlock({ text }: { text: string }) {
  return <div className={styles.markdownBlock} dangerouslySetInnerHTML={{ __html: addModelMarks(renderMarkdown(text,
    { pageId: 'agent', resolveDocumentLink: () => undefined, renderImages: true })) }} />
})

// Preserve the browser's text selection when the quote button or conversation status updates.
const ReplyText = memo(function ReplyText({ id, text, failureReason, locale = 'zh' }: { id: string; text: string; failureReason?: string; locale?: Locale }) {
  const [preview, setPreview] = useState<string | null>(null)
  // Old failed turns retain the SDK exception; show it as a diagnostic, not Markdown prose.
  if (failureReason && /^(?:本轮处理失败：)?Unknown content(?::| type in ResponseOutputMessage:)/.test(failureReason)) {
    return <div className={styles.parseFailure}>
      <p>{locale === 'zh' ? '对话内容解析失败，本轮未完成。请重试。' : 'This turn could not parse the conversation. Please try again.'}</p>
      <details><summary>{locale === 'zh' ? '错误详情' : 'Error details'}</summary><pre>{failureReason}</pre></details>
    </div>
  }
  return <>
    <div className={`${styles.richText} selectable`} data-quote-source={id} onClick={event => {
      const target = event.target
      if (target instanceof HTMLImageElement && target.classList.contains('markdown-image')) setPreview(target.currentSrc || target.src)
    }}>{replyMarkdownBlocks(text).map((block, index) => <ReplyBlock key={index} text={block} />)}</div>
    {preview && <div className={styles.imageLightbox} role="dialog" aria-label="查看图片" onClick={() => setPreview(null)}>
      <button type="button" aria-label="关闭图片" onClick={() => setPreview(null)}><X size={20} /></button>
      <img src={preview} alt="" onClick={event => event.stopPropagation()} />
    </div>}
  </>
})

function WorkTime({ turn, serverTime, locale }: { turn?: ConversationTurn; serverTime?: number; locale: Locale }) {
  const end = turn?.finished_at ?? (turn?.state === 'running' ? serverTime : undefined)
  if (turn?.started_at == null || end == null) return null
  const seconds = Math.max(0, end - turn.started_at)
  const duration = seconds < 60 ? `${turn.state === 'running' ? Math.floor(seconds) : seconds.toFixed(1)} ${locale === 'zh' ? '秒' : 's'}`
    : `${Math.floor(seconds / 60)} ${locale === 'zh' ? '分' : 'min'} ${Math.floor(seconds % 60)} ${locale === 'zh' ? '秒' : 's'}`
  return <span className={styles.workTime} data-running={turn.state === 'running'} aria-live="off"
    title={locale === 'zh' ? '从服务端受理到本轮结束，包含工具执行和等待确认' : 'From acceptance to completion, including tools and approval waits'}>
    {locale === 'zh' ? turn.state === 'running' ? '已用时 ' : '用时 ' : 'Worked for '}{duration}
  </span>
}

function ReplyQuote({ message, quote, locale, disabled, onClear }: {
  message?: ConversationMessage; quote?: string; locale: Locale; disabled?: boolean; onClear?: () => void
}) {
  return <blockquote className={styles.replyQuote} data-composer={Boolean(onClear)} aria-label={locale === 'zh' ? '引用的回复' : 'Quoted reply'}>
    <CornerUpLeft size={13} className={styles.quoteMark} aria-hidden="true" />
    <details className={styles.quoteContent}>
      <summary title={locale === 'zh' ? '展开或收起引用' : 'Expand or collapse quote'}>
        <div className={styles.quoteSummary}>
          {quote ? <p className={styles.quoteText}>{quote}</p> : message ? <div className={`${styles.richText} ${styles.quoteText}`} dangerouslySetInnerHTML={{ __html: renderMarkdown(message.text,
            { pageId: 'quote', resolveDocumentLink: () => undefined }) }} />
            : <p>{locale === 'zh' ? '已引用一条回复' : 'A reply is quoted'}</p>}
        </div>
        <ChevronDown size={12} aria-hidden="true" />
      </summary>
    </details>
    {onClear && <button type="button" className={styles.iconButton} onClick={onClear} disabled={disabled}
      aria-label={locale === 'zh' ? '取消引用' : 'Remove quote'}><X size={13} /></button>}
  </blockquote>
}

function ReplyMeta({ message, turn, locale, disabled, regenerate, regenerateDisabled, fork, forkDisabled, onFollowup }: {
  message: ConversationMessage; turn?: ConversationTurn; locale: Locale; disabled: boolean
  regenerate?: () => void; regenerateDisabled: boolean; onFollowup: (prompt?: string) => void
  fork?: () => void; forkDisabled?: boolean
}) {
  const [copyState, setCopyState] = useState('')
  useEffect(() => {
    if (copyState !== 'done') return
    const timer = setTimeout(() => setCopyState(''), 1800)
    return () => clearTimeout(timer)
  }, [copyState])
  const stamp = message.created_at == null ? null : new Date(message.created_at * 1000)
  return <footer className={styles.replyMeta}>
    <button type="button" className={styles.copyReply} aria-label={locale === 'zh' ? '复制回复' : 'Copy reply'}
      data-copied={copyState === 'done'} onClick={async () => {
        try { await navigator.clipboard.writeText(message.text); setCopyState('done') }
        catch { setCopyState('failed') }
      }} title={locale === 'zh' ? '复制回复' : 'Copy reply'}>{copyState === 'done' ? <Check size={14} /> : <Copy size={14} />}</button>
    <button type="button" className={styles.copyReply} disabled={disabled} onClick={() => onFollowup()}
      aria-label={locale === 'zh' ? '引用追问' : 'Follow up'} title={locale === 'zh' ? '引用追问' : 'Follow up'}><CornerUpLeft size={14} /></button>
    {regenerate && <button type="button" className={styles.copyReply} disabled={regenerateDisabled} onClick={regenerate}
      aria-label={locale === 'zh' ? '重新生成回答' : 'Regenerate reply'} title={regenerateDisabled
        ? locale === 'zh' ? '请先完成当前回合并清空草稿' : 'Finish the current turn and clear the draft first'
        : locale === 'zh' ? '重新生成回答 · 只读，不重复动作' : 'Regenerate · read only, no repeated actions'}><RotateCw size={14} /></button>}
    {fork && <button type="button" className={styles.copyReply} disabled={forkDisabled} onClick={fork}
      aria-label={locale === 'zh' ? '从这里开分支' : 'Branch from here'}
      title={locale === 'zh' ? '从这里开分支 · 保留至这条回复的对话' : 'Start a new conversation with history through this reply'}><GitBranch size={14} aria-hidden="true" /></button>}
    <MessageReactions />
    <div className={styles.replyTime}>
      {turn?.state !== 'running' && <WorkTime turn={turn} locale={locale} />}
      {stamp && <time dateTime={stamp.toISOString()} title={stamp.toLocaleString(locale === 'zh' ? 'zh-CN' : 'en-US')}>
        {stamp.toLocaleTimeString(locale === 'zh' ? 'zh-CN' : 'en-US', { hour: '2-digit', minute: '2-digit', hour12: false })}
      </time>}
    </div>
    {copyState && <span className={copyState === 'done' ? styles.screenReaderOnly : undefined} role="status" data-error={copyState === 'failed'}>{copyState === 'done'
      ? locale === 'zh' ? '已复制' : 'Copied'
      : locale === 'zh' ? '复制失败，请选中文字复制' : 'Copy failed. Select the text to copy.'}</span>}
  </footer>
}


function ReasoningIcon({ value }: { value: string }) {
  if (value === 'off') return <Zap size={14} aria-hidden="true" />
  if (value === 'on') return <BrainCircuit size={14} aria-hidden="true" />
  const level = ['low', 'medium', 'high', 'xhigh', 'max'].indexOf(value)
  return level >= 0 ? <EffortGlyph count={5} filled={level + 1} /> : <SlidersHorizontal size={14} aria-hidden="true" />
}

function ComposerMenu({ label, value, options, disabled, reasoning = false, permission = false, placeholder, onChange }: {
  label: string
  value: string
  options: { id: string; label: string; provider?: string; configured?: boolean; key_env?: string }[]
  disabled: boolean
  reasoning?: boolean
  permission?: boolean
  placeholder?: string
  onChange: (value: string) => void
}) {
  const menu = useRef<HTMLDetailsElement>(null)
  const selected = options.find(option => option.id === value)
  const providerIcons: Record<string, string> = { openai: 'openai.svg', codex: 'openai.svg', anthropic: 'anthropic.png', claude: 'anthropic.png', moonshot: 'moonshot.ico', qwen: 'qwen.png',
    google: 'google-gemini.png', gemini: 'google-gemini.png', deepseek: 'deepseek.ico', grok: 'grok.ico', mistral: 'mistral.ico',
    meta: 'meta.ico', perplexity: 'perplexity.ico', minimax: 'minimax.ico', zhipu: 'zhipu.png', cohere: 'cohere.ico', baidu: 'baidu.ico' }
  return <details ref={menu} className={`${styles.selectMenu} ${reasoning ? styles.reasoningMenu : permission ? styles.permissionMenu : styles.modelMenu}`}
    name="lingtu-menu" onBlur={event => {
      if (event.relatedTarget && !event.currentTarget.contains(event.relatedTarget)) event.currentTarget.open = false
    }} onKeyDown={event => {
      if (disabled) return
      const element = event.currentTarget
      if (event.key === 'Escape' && element.open) {
        event.preventDefault()
        event.stopPropagation()
        element.open = false
        element.querySelector('summary')?.focus()
        return
      }
      if (!['ArrowDown', 'ArrowUp', 'Home', 'End'].includes(event.key)) return
      event.preventDefault()
      element.open = true
      const buttons = [...element.querySelectorAll<HTMLButtonElement>('[role="menuitemradio"]:not(:disabled)')]
      const index = buttons.indexOf(document.activeElement as HTMLButtonElement)
      let next = index + (event.key === 'ArrowDown' ? 1 : -1)
      if (index < 0 || event.key === 'Home') next = 0
      if (event.key === 'End' || (index < 0 && event.key === 'ArrowUp')) next = buttons.length - 1
      buttons[(next + buttons.length) % buttons.length]?.focus()
    }}>
    <summary className={`${styles.menuTrigger} ${permission && value === 'full_access' ? styles.fullAccess : ''} ${reasoning && !['default', 'off'].includes(value) ? styles.reasoningActive : ''}`}
      aria-label={label} aria-haspopup="menu" aria-disabled={disabled}
      tabIndex={disabled ? -1 : 0} title={selected?.label ?? placeholder}
      onClick={event => { if (disabled) event.preventDefault() }}>
      {reasoning && <ReasoningIcon value={value} />}
      {permission && <ShieldCheck size={14} className={styles.permissionIcon} data-mode={value} aria-hidden="true" />}
      {!(reasoning && value === 'default') && <span>{selected?.label ?? placeholder ?? '—'}</span>}<ChevronDown size={12} aria-hidden="true" />
    </summary>
    <div className={styles.selectPanel} role="menu" aria-label={label}>
      <div className={styles.menuHeading}>{label}</div>
      {options.map(option => <button type="button" key={option.id} role="menuitemradio" aria-checked={option.id === value}
        disabled={disabled || option.configured === false} title={option.configured === false ? option.provider === 'codex' ? '请在管理模型连接中登录' : `服务端需配置 ${option.key_env || '模型连接'}` : undefined} onClick={() => {
          onChange(option.id)
          if (menu.current) {
            menu.current.open = false
            menu.current.querySelector('summary')?.focus()
          }
        }}>
        {reasoning && <ReasoningIcon value={option.id} />}
        <span>{option.label}</span>
        {option.configured === false && <small className={styles.unconfigured}>未配置</small>}
        {option.id === value && <Check size={13} aria-hidden="true" />}
        {option.provider && providerIcons[option.provider] && <img className={styles.providerIcon}
          src={`/assets/providers/${providerIcons[option.provider]}`} alt={option.provider} width={16} height={16} />}
      </button>)}
    </div>
  </details>
}

function ConversationSessions({ locale, controller: conversation, onShare }: AgentConversationProps & { onShare(): void }) {
  const [editing, setEditing] = useState<{ id: string; title: string } | null>(null)
  const text = (en: string, zh: string) => locale === 'zh' ? zh : en
  const currentId = conversation.snapshot?.conversation_id
  const sessions = conversation.conversations ?? []
  const busy = Boolean(conversation.sessionBusy || conversation.shareBusy || conversation.sending || conversation.uploading || conversation.approvalBusy)
  const searching = Boolean(conversation.searchQuery?.trim())

  async function switchTo(id: string) {
    try {
      await conversation.switchConversation(id)
      conversation.setSessionListOpen(false)
    } catch { /* The controller keeps the request error visible. */ }
  }

  return <section className={styles.sessionsPanel} aria-label={text('Conversations', '会话管理')} aria-busy={conversation.sessionsLoading || busy}>
    <header className={styles.sessionsHeading}>
      <strong>{conversation.archivedView ? text('Archived', '已归档') : text('Conversations', '历史会话')}</strong>
      <button type="button" className={styles.iconButton} aria-pressed={conversation.archivedView}
        aria-label={text('Show archived conversations', '查看归档会话')} onClick={() => {
          setEditing(null)
          conversation.setArchivedView(!conversation.archivedView)
        }}><Archive size={16} /></button>
      <button type="button" className={styles.iconButton} disabled={busy} aria-label={text('New conversation', '新建会话')}
        onClick={async () => {
          try { await conversation.createNewConversation(); conversation.setSessionListOpen(false) }
          catch { /* The controller preserves the previous conversation. */ }
        }}><Plus size={18} /></button>
      <button type="button" className={styles.iconButton} aria-label={text('Close conversation list', '收起会话列表')}
        onClick={() => conversation.setSessionListOpen(false)}><X size={17} /></button>
    </header>
    <div className={styles.sessionSearch}>
      <Search size={15} aria-hidden="true" />
      <input type="search" value={conversation.searchQuery ?? ''} maxLength={200}
        aria-label={text('Search conversation contents', '搜索会话正文')} placeholder={text('Search conversations…', '搜索对话内容…')}
        onChange={event => conversation.setSearchQuery(event.target.value)} />
      {conversation.searchQuery && <button type="button" className={styles.iconButton} aria-label={text('Clear search', '清空搜索')}
        onClick={() => conversation.setSearchQuery('')}><X size={14} aria-hidden="true" /></button>}
    </div>
    <div className={styles.sessionEntries} aria-busy={conversation.searchBusy || undefined}>
      {searching ? <>
        {conversation.searchBusy && <p className={styles.sessionEmpty} role="status">{text('Searching…', '正在搜索…')}</p>}
        {conversation.searchError && <p className={styles.sessionEmpty} role="alert">{conversation.searchError}</p>}
        {!conversation.searchBusy && !conversation.searchError && !conversation.searchResults?.length && <p className={styles.sessionEmpty} role="status">
          {text('No matching messages', '没有找到匹配的消息')}</p>}
        {conversation.searchResults?.map(result => <button type="button" key={`${result.conversation_id}:${result.message_id}`}
          className={styles.searchResult} disabled={busy} onClick={() => { void conversation.openSearchResult(result).catch(() => {}) }}>
          <strong>{result.title || text('Conversation', '对话')}</strong>
          <span>{result.snippet}</span>
          <small>{result.role === 'user' ? text('You', '你') : 'LingTu'}</small>
        </button>)}
      </> : <>
      {!sessions.length && <p className={styles.sessionEmpty}>{conversation.sessionsLoading
        ? text('Loading…', '读取中…') : text('No conversations', '暂无会话')}</p>}
      {sessions.map(session => <div key={session.conversation_id} className={styles.sessionRow}
        data-session-id={session.conversation_id} data-current={session.conversation_id === currentId}>
        {editing?.id === session.conversation_id ? <form className={styles.renameForm} onSubmit={async event => {
          event.preventDefault()
          if (!editing.title.trim()) return
          try { await conversation.renameConversation(editing.id, editing.title.trim()); setEditing(null) }
          catch { /* Keep the edited title available for retry. */ }
        }}>
          <input autoFocus aria-label={text('Conversation title', '会话名称')} maxLength={120} value={editing.title}
            onChange={event => setEditing({ ...editing, title: event.target.value })}
            onKeyDown={event => { if (event.key === 'Escape') { event.preventDefault(); setEditing(null) } }} />
          <button type="submit" className={styles.iconButton} disabled={busy || !editing.title.trim()}
            aria-label={text('Save title', '保存名称')}><Check size={16} /></button>
          <button type="button" className={styles.iconButton} onClick={() => setEditing(null)}
            aria-label={text('Cancel rename', '取消重命名')}><X size={16} /></button>
        </form> : <>
          <button type="button" className={styles.sessionSelect} disabled={busy}
            aria-current={session.conversation_id === currentId ? 'true' : undefined} onClick={() => { void switchTo(session.conversation_id) }}>
            <MessageCircle size={15} aria-hidden="true" /><span>{session.title || text('New conversation', '新会话')}</span>
            {session.conversation_id === currentId && <Check size={14} aria-hidden="true" />}
          </button>
          <details className={styles.sessionActions} name="lingtu-menu">
            <summary className={styles.iconButton} aria-label={text(`Actions for ${session.title}`, `${session.title}的更多操作`)}
              aria-haspopup="menu"><Ellipsis size={17} /></summary>
            <div className={styles.sessionActionPanel} role="menu" aria-label={text('Conversation actions', '会话操作')}>
              <button type="button" role="menuitem" disabled={busy} onClick={event => {
                event.currentTarget.closest('details')?.removeAttribute('open')
                setEditing({ id: session.conversation_id, title: session.title })
              }}><Pencil size={15} />{text('Rename', '重命名')}</button>
              {session.conversation_id === currentId && <button type="button" role="menuitem" disabled={busy} onClick={event => {
                const menu = event.currentTarget.closest('details')
                menu?.removeAttribute('open')
                menu?.querySelector('summary')?.focus()
                onShare()
              }}><Share2 size={15} />{text('Share', '分享')}</button>}
              <button type="button" role="menuitem" disabled={busy} onClick={async event => {
                event.currentTarget.closest('details')?.removeAttribute('open')
                try { await conversation.archiveConversation(session.conversation_id, !session.archived) }
                catch { /* The controller reports archive failures. */ }
              }}>{session.archived ? <ArchiveRestore size={15} /> : <Archive size={15} />}{session.archived ? text('Restore', '恢复') : text('Archive', '归档')}</button>
            </div>
          </details>
        </>}
      </div>)}
      </>}
    </div>
    {conversation.sessionBusy && <p className={styles.sessionFeedback} role="status">{text('Processing…', '处理中…')}</p>}
  </section>
}

export function AgentConversation({ locale, controller: conversation }: AgentConversationProps) {
  const openImu = useImuMonitor()
  const openSensor = useSensorMonitor()
  const monitorSeen = useRef<Set<string> | null>(null)
  useEffect(() => {
    if (!conversation.snapshot) return
    const messages = conversation.snapshot.messages
    if (monitorSeen.current === null) { monitorSeen.current = new Set(messages.filter(item => item.metadata?.status !== 'running').map(item => item.id)); return }
    for (const message of messages) {
      if (message.role !== 'tool' || message.metadata?.status === 'running' || monitorSeen.current.has(message.id)) continue
      monitorSeen.current.add(message.id)
      try {
        const result = JSON.parse(message.text)
        if (result.success !== true) continue
        if (result.monitor === 'imu') openImu?.()
        else if (result.monitor === 'catalog') openSensor?.('catalog')
        else { const card = sensorCards.find(item => item.id === result.monitor); if (card) openSensor?.(card.id) }
      }
      catch { /* Plain tool output cannot request a panel. */ }
    }
  }, [conversation.snapshot, openImu, openSensor])
  const [botProfile, setBotProfile] = useState(readBotProfile)
  const [botStudioOpen, setBotStudioOpen] = useState(false)
  const [connectionsOpen, setConnectionsOpen] = useState(false)
  const mapSettings = useMapSettings()
  const [expanded, setExpanded] = useState(false)
  const [sharingId, setSharingId] = useState<string | null>(null)
  const currentId = conversation.snapshot?.conversation_id
  useEffect(() => { setSharingId(null) }, [currentId])
  const [anchorId, setAnchorId] = useState<string | null>(null)
  const root = useRef<HTMLElement>(null)
  const historyEnd = useRef<HTMLDivElement>(null)
  const locatedSearch = useRef<unknown>(null)
  const draftInput = useRef<HTMLTextAreaElement>(null)
  const fileInput = useRef<HTMLInputElement>(null)
  const [selectionQuote, setSelectionQuote] = useState<{ messageId: string; text: string; left: number; top: number } | null>(null)
  useEffect(() => {
    if (!root.current) return
    const element = root.current
    let dragging = false
    function selectionChanged() {
      if (dragging) return
      const selection = window.getSelection()
      const range = selection?.rangeCount ? selection.getRangeAt(0) : null
      const start = range?.startContainer
      const source = (start instanceof Element ? start : start?.parentElement)?.closest<HTMLElement>('[data-quote-source]')
      const selected = selection?.toString().trim()
      if (!range || !selected || !source || !root.current?.contains(source) || !source.contains(range.endContainer)) {
        setSelectionQuote(null)
        return
      }
      const rect = range.getBoundingClientRect()
      const bounds = root.current.getBoundingClientRect()
      setSelectionQuote({ messageId: source.dataset.quoteSource!, text: selected,
        left: Math.max(68, Math.min(rect.left + rect.width / 2 - bounds.left, bounds.width - 68)),
        top: Math.max(8, rect.top - bounds.top - 38) })
    }
    function startSelection(event: PointerEvent) {
      if (!(event.target instanceof Element) || !event.target.closest('[data-quote-source]')) return
      dragging = true
      setSelectionQuote(null)
    }
    function endSelection() {
      if (!dragging) return
      dragging = false
      selectionChanged()
    }
    function dismissSelection(event: KeyboardEvent) {
      if (event.key === 'Escape') setSelectionQuote(null)
    }
    element.addEventListener('pointerdown', startSelection)
    document.addEventListener('pointerup', endSelection)
    document.addEventListener('pointercancel', endSelection)
    document.addEventListener('keydown', dismissSelection)
    document.addEventListener('selectionchange', selectionChanged)
    return () => {
      element.removeEventListener('pointerdown', startSelection)
      document.removeEventListener('pointerup', endSelection)
      document.removeEventListener('pointercancel', endSelection)
      document.removeEventListener('keydown', dismissSelection)
      document.removeEventListener('selectionchange', selectionChanged)
    }
  }, [])
  const followLatest = useRef(true)
  const historyViewport = useRef<HTMLDivElement>(null)
  const historyPositions = useRef(new Map<string, { top: number; follow: boolean }>())
  const latestTimer = useRef<ReturnType<typeof setTimeout> | undefined>(undefined)
  const [awayConversation, setAwayConversation] = useState<string | null>(null)
  const showLatest = awayConversation === (conversation.snapshot?.conversation_id ?? '')
  const text = (en: string, zh: string) => locale === 'zh' ? zh : en
  const messages = conversation.snapshot?.messages ?? EMPTY_MESSAGES
  const queuedTurnIds = new Set(conversation.snapshot?.turns.filter(turn => turn.state === 'queued').map(turn => turn.turn_id))
  const visibleMessages = messages.filter(message => !queuedTurnIds.has(message.turn_id))
  const messageGroups: ConversationMessage[][] = []
  for (const message of visibleMessages) {
    const previous = messageGroups.at(-1)
    if (message.role === 'tool' && previous?.[0].role === 'tool' && previous[0].turn_id === message.turn_id) previous.push(message)
    else messageGroups.push([message])
  }
  const seenToolResults = useRef<Set<string> | null>(null)
  const refreshMap = mapSettings?.map?.onChanged
  useEffect(() => {
    const seen = seenToolResults.current
    let changed = false
    if (seen) for (const message of messages) {
      if (seen.has(`${message.id}:${message.metadata?.status}`) || message.role !== 'tool'
        || !['save_map_copy', 'set_navigation_region', 'save_inspection_point'].includes(message.kind ?? '')) continue
      try {
        const result = JSON.parse(message.text)
        changed ||= result.success === true || result.ok === true
      } catch { /* Plain tool messages cannot confirm a saved change. */ }
    }
    seenToolResults.current = new Set(messages.map(message => `${message.id}:${message.metadata?.status}`))
    if (changed) refreshMap?.()
  }, [messages, refreshMap])
  const turns = conversation.snapshot?.turns ?? []
  const turnsById = new Map(turns.map(turn => [turn.turn_id, turn]))
  const unreportedTurns = turns.filter(turn => turn.kind !== 'queue' && ['failed', 'cancelled', 'interrupted'].includes(turn.state)
    && !messages.some(message => message.turn_id === turn.turn_id && message.role === 'assistant'))
  const lastReplies = new Map(messages.filter(message => message.role === 'assistant').map(message => [message.turn_id, message.id]))
  const latestReply = messages.filter(message => message.role === 'assistant').at(-1)
  const approvals = conversation.snapshot?.approvals?.filter(approval => approval.state === 'pending') ?? []
  const initialLoading = !conversation.snapshot && conversation.loading
  const archived = conversation.snapshot?.metadata?.archived === true
  const showHistory = !conversation.sessionListOpen && conversation.open && (messages.length > 0 || turns.length > 0 || conversation.active || initialLoading || approvals.length > 0 || archived)
  const model = conversation.options?.models.find(item => item.id === conversation.model)
  const controlsDisabled = !conversation.options || !conversation.editable || conversation.sending || conversation.uploading
    || Boolean(conversation.sessionBusy) || archived
  const regenerateDisabled = controlsDisabled || Boolean(conversation.active || conversation.pending || conversation.snapshot?.active_run
    || approvals.length || conversation.replyTo || conversation.draft.trim() || conversation.attachments.length)
  function quoteReply(messageId: string, quote = '') {
    conversation.setReplyTo(messageId, quote)
    window.getSelection()?.removeAllRanges()
    setSelectionQuote(null)
    draftInput.current?.focus()
  }
  const runningTurn = turns.find(turn => turn.state === 'running')
  const currentTools = runningTurn ? messages.filter(message => message.role === 'tool' && message.turn_id === runningTurn.turn_id) : []
  const currentTool = currentTools.findLast(message => message.metadata?.status === 'running') ?? currentTools.at(-1)
  const toolRunning = Boolean(currentTool && (currentTool.metadata?.status === 'running'
    || currentTool.kind === 'tool_start' && currentTool.metadata?.status == null))
  const currentReply = runningTurn ? messages.filter(message => message.role === 'assistant' && message.turn_id === runningTurn.turn_id).at(-1) : undefined
  const writingReply = currentReply?.metadata?.status === 'streaming'
  const activeTurn = Boolean(runningTurn || conversation.active)
  let workingLabel = text('Preparing a reply', '正在准备回复')
  if (toolRunning) {
    workingLabel = text('Running a tool', '正在调用工具')
    if (currentTool?.kind === 'tool_start') {
      try { const name = JSON.parse(currentTool.text).tool; if (typeof name === 'string') workingLabel = `${text('Running', '正在调用')} · ${toolNames[name] ?? name}` }
      catch { /* A legacy start can omit a parseable tool name. */ }
    } else if (currentTool?.kind) workingLabel = `${text('Running', '正在调用')} · ${toolNames[currentTool.kind] ?? currentTool.kind}`
  } else if (writingReply) workingLabel = text('Writing a reply', '正在回复')
  else if (currentTool) workingLabel = text('Processing tool results', '正在处理工具结果')
  const latestTurn = turns.at(-1)
  const workStatus = approvals.length ? `${text('Awaiting approval', '等待确认')} · ${toolNames[approvals[0].tool_name] ?? approvals[0].tool_name}`
    : conversation.optionsLoading ? text('Connecting', '正在连接')
      : !conversation.options || conversation.optionsError ? text('Chat is unavailable', '对话暂不可用')
        : conversation.error ? text('Connection interrupted', '连接异常，请重试')
          : conversation.uploading ? text('Uploading attachments', '正在上传附件')
            : conversation.sending ? text('Sending message', '正在发送')
              : initialLoading ? text('Loading conversation', '正在读取会话')
                : conversation.sessionBusy ? text('Updating conversation', '正在处理会话')
                  : activeTurn ? workingLabel
                    : latestTurn?.state === 'failed' ? text('This turn failed', '本轮未完成')
                      : latestTurn?.state === 'interrupted' ? text('This turn was interrupted', '本轮已中断')
                        : latestTurn?.state === 'cancelled' ? text('This turn was cancelled', '本轮已取消') : ''
  const botState: BotState = approvals.length ? 'waiting' : conversation.optionsLoading ? 'processing'
    : !conversation.options ? 'unavailable' : conversation.error || conversation.optionsError ? 'error'
      : conversation.uploading || conversation.sending || initialLoading || conversation.sessionBusy ? 'processing'
        : activeTurn ? toolRunning ? 'working' : writingReply ? 'writing' : 'processing'
          : latestTurn?.state === 'failed' ? 'error' : latestTurn?.state === 'completed' ? 'done' : 'idle'
  const botLabel = workStatus || (botState === 'processing' ? text('Processing', '处理中')
    : botState === 'working' ? text('Running a tool', '正在调用工具') : botState === 'writing' ? text('Writing a reply', '正在回复')
      : botState === 'waiting' ? text('Waiting for you', '等待你的确认')
      : botState === 'unavailable' ? text('Chat is unavailable', '对话暂不可用')
        : botState === 'error' ? text('Unable to continue', '暂时无法继续')
        : botState === 'done' ? text('Ready to continue', '可以继续聊') : botProfile.name)
  useMenuDismiss(root)
  const fileSize = (size: number) => size >= 1024 * 1024 ? `${(size / 1024 / 1024).toFixed(1)} MB` : `${Math.ceil(size / 1024)} KB`
  useLayoutEffect(() => {
    const id = conversation.snapshot?.conversation_id
    const viewport = historyViewport.current
    if (!id || !viewport) return
    const saved = historyPositions.current.get(id)
    followLatest.current = saved?.follow ?? true
    viewport.scrollTop = followLatest.current ? viewport.scrollHeight : saved?.top ?? 0
    const frame = requestAnimationFrame(() => setAwayConversation(followLatest.current ? null : id))
    const positions = historyPositions.current
    return () => {
      positions.set(id, { top: viewport.scrollTop, follow: followLatest.current })
      cancelAnimationFrame(frame)
      clearTimeout(latestTimer.current)
    }
  }, [conversation.snapshot?.conversation_id, showHistory])
  useLayoutEffect(() => {
    const viewport = historyViewport.current
    if (!viewport || !conversation.open) return
    const follow = () => { if (followLatest.current) viewport.scrollTop = viewport.scrollHeight }
    follow()
    const observer = new ResizeObserver(follow)
    for (const child of viewport.children) observer.observe(child)
    return () => observer.disconnect()
  }, [conversation.open, messages, showHistory])
  useEffect(() => {
    const target = conversation.searchTarget
    if (!showHistory || !target || target.conversationId !== currentId || locatedSearch.current === target) return
    const element = [...(root.current?.querySelectorAll<HTMLElement>('[data-message-id]') ?? [])]
      .find(item => item.dataset.messageId === target.messageId)
    if (!element) return
    locatedSearch.current = target
    followLatest.current = false
    setAwayConversation(target.conversationId)
    element.scrollIntoView({ block: 'center', behavior: window.matchMedia('(prefers-reduced-motion: reduce)').matches ? 'instant' : 'smooth' })
    element.focus({ preventScroll: true })
  }, [conversation.searchTarget, currentId, messages, showHistory])
  useEffect(() => {
    const input = draftInput.current
    if (!input) return
    input.style.height = 'auto'
    input.style.height = `${Math.max(32, Math.min(input.scrollHeight, 120))}px`
  }, [conversation.draft])

  const approvalAction = (name: string) => ({
    navigate_to: text('Navigate to position', '前往目标位置'),
    navigate_to_object: text('Navigate to object', '前往目标物体'),
    navigate_to_deg: text('Turn to heading', '转向目标方向'),
    find_object: text('Find object', '查找物体'),
    ...toolNames,
  } as Record<string, string>)[name] ?? name
  const approvalArguments = (arguments_: Record<string, unknown>) => Object.entries(arguments_)
    .map(([name, value]) => `${name} ${typeof value === 'string' ? value : JSON.stringify(value)}`).join(' · ')

  const kindLabel = (kind?: string) => {
    if (kind === 'clarification') return text('Clarification', '需要补充信息')
    if (kind === 'task_complete') return text('Execution result', '执行结果')
    if (kind === 'failed') return text('Unable to continue', '未能继续')
    if (kind === 'control') return text('Task control', '任务控制')
    if (kind === 'task_active') return text('Current task', '当前任务')
    return text('Reply', '回答')
  }

  return (
    <section ref={root} className={`${styles.conversation} ${mapSettings?.editing ? styles.editing : ''} ${expanded && conversation.open ? styles.expanded : ''}`} data-agent-conversation aria-label={text('Robot conversation', '机器人对话')}>
      {selectionQuote && conversation.open && !conversation.sessionListOpen && !controlsDisabled && !conversation.pending
        && <div className={styles.selectionQuote} role="toolbar" aria-label={text('Selected text', '选中文字操作')}
          style={{ left: selectionQuote.left, top: selectionQuote.top }}>
          <button type="button" aria-label={text('Quote selection', '引用选中文字')}
            onPointerDown={event => event.preventDefault()} onClick={() => quoteReply(selectionQuote.messageId, selectionQuote.text)}>
            <CornerUpLeft size={13} aria-hidden="true" /><span>{text('Add to chat', '引用到对话')}</span></button>
        </div>}
      {conversation.sessionListOpen && conversation.options && <ConversationSessions locale={locale} controller={conversation}
        onShare={() => setSharingId(currentId ?? null)} />}
      {!conversation.sessionListOpen && !showHistory && <div className={styles.idleHeading}>
        <button type="button" className={styles.iconButton} disabled={!conversation.options}
          aria-label={text('Conversation history', '会话历史')} onClick={() => { conversation.setOpen(true); conversation.setSessionListOpen(true) }}>
          <History size={16} aria-hidden="true" />
        </button>
        <button type="button" className={styles.botEntry} onClick={() => setBotStudioOpen(true)} aria-label="打开 Bot 形象与角色" title={`${botProfile.name} · 形象、角色与权限`}>
          <Bot state={botState} shape={botProfile.shape} color={botProfile.color} size={botProfile.size} />
        </button>
      </div>}
      {showHistory && (
        <div className={styles.history}>
          <header className={styles.heading}>
            <button type="button" className={styles.conversationTitle} aria-expanded={Boolean(conversation.sessionListOpen)}
              aria-label={text('Conversation history', '会话历史')} title={text('Conversation history', '会话历史')}
              onClick={() => conversation.setSessionListOpen(!conversation.sessionListOpen)}>
              <History size={16} aria-hidden="true" /><span>{conversation.snapshot?.metadata?.title || text('Conversation', '对话')}</span>
              <ChevronDown size={13} aria-hidden="true" /></button>
            <button type="button" className={styles.botEntry} onClick={() => setBotStudioOpen(true)} aria-label="打开 Bot 形象与角色" title={`${botProfile.name} · 形象、角色与权限`}>
              <Bot state={botState} shape={botProfile.shape} color={botProfile.color} size={botProfile.size} />
            </button>
            <button type="button" className={styles.iconButton} aria-label={text('Share conversation', '分享对话')}
              title={text('Preview and share this conversation', '预览并分享当前会话')}
              disabled={!currentId || initialLoading || Boolean(conversation.sessionBusy || conversation.shareBusy || conversation.sending || conversation.uploading || conversation.approvalBusy)}
              onClick={() => setSharingId(currentId ?? null)}><Share2 size={16} aria-hidden="true" /></button>
            <button type="button" className={styles.iconButton} aria-label={expanded ? '收起对话工作区' : '展开对话工作区'}
              aria-pressed={expanded} onClick={() => setExpanded(!expanded)}>{expanded ? <Minimize2 size={15} /> : <Maximize2 size={15} />}</button>
            {archived && <span className={styles.archivedLabel}>{text('Archived', '已归档')}</span>}
            <button type="button" className={styles.iconButton} onClick={() => conversation.setOpen(false)}
              aria-label={text('Close conversation', '收起对话')}><X size={18} /></button>
          </header>
          {conversation.snapshot?.metadata?.forked_from && <div className={styles.forkOrigin}>
            <GitBranch size={13} aria-hidden="true" />
            <span>{text('Branched from', '分支自')} · <strong>{conversation.snapshot.metadata.forked_from.title}</strong></span>
            <button type="button" disabled={Boolean(conversation.sessionBusy || conversation.sending || conversation.uploading || conversation.approvalBusy || conversation.pending)}
              onClick={() => {
                const origin = conversation.snapshot!.metadata!.forked_from!
                void conversation.openSearchResult({ ...origin, role: 'assistant', snippet: '' }).catch(() => {})
              }}>{text('Original conversation', '回到原会话')}<ArrowUpRight size={12} aria-hidden="true" /></button>
          </div>}
          {visibleMessages.filter(message => message.role === 'user').length > 1 && <nav className={styles.conversationRail} aria-label="对话定位">
            {visibleMessages.filter(message => message.role === 'user').map((message, index) => <button type="button" key={message.id}
              aria-label={`跳到第 ${index + 1} 次提问：${message.text.slice(0, 60)}`} title={message.text.slice(0, 100)}
              aria-current={anchorId === message.id ? 'step' : undefined} onClick={() => {
                setAnchorId(message.id)
                followLatest.current = false
                setAwayConversation(conversation.snapshot?.conversation_id ?? '')
                const target = [...(root.current?.querySelectorAll<HTMLElement>('[data-message-id]') ?? [])]
                  .find(element => element.dataset.messageId === message.id)
                target?.scrollIntoView({ block: 'start', behavior: window.matchMedia('(prefers-reduced-motion: reduce)').matches ? 'instant' : 'smooth' })
              }}><span /></button>)}
          </nav>}
          <div ref={historyViewport} className={styles.messages} role="log" aria-live="polite" aria-relevant="additions text"
            onScroll={event => {
              setSelectionQuote(null)
              const { scrollHeight, scrollTop, clientHeight } = event.currentTarget
              followLatest.current = scrollHeight - scrollTop - clientHeight <= 48
              clearTimeout(latestTimer.current)
              if (followLatest.current) setAwayConversation(null)
              else latestTimer.current = setTimeout(() => setAwayConversation(conversation.snapshot?.conversation_id ?? ''), 150)
            }}
            hidden={!messages.length && !turns.length && !conversation.active && !initialLoading && !approvals.length}>
            {initialLoading && <p className={styles.empty}>{text('Loading…', '读取中…')}</p>}
            {conversation.snapshot?.history_cursor != null && <button type="button" className={styles.iconButton}
              disabled={conversation.historyLoading} onClick={async () => {
                followLatest.current = false
                const viewport = historyViewport.current
                const height = viewport?.scrollHeight ?? 0
                await conversation.loadHistory()
                requestAnimationFrame(() => {
                  if (viewport?.isConnected) viewport.scrollTop += viewport.scrollHeight - height
                })
              }}>
              {conversation.historyLoading ? '正在读取…' : '更早的消息'}</button>}
            {messageGroups.map(group => { const message = group[0]; return <article key={message.id}
              className={`${styles.message} ${message.role === 'user' ? styles.user : message.role === 'tool' ? styles.tool : ''}`}
              data-message-id={message.id} data-kind={message.kind} tabIndex={-1}
              data-search-match={conversation.searchTarget?.conversationId === currentId && conversation.searchTarget?.messageId === message.id || undefined}
              aria-label={message.role === 'user' ? text('You', '你')
                : message.role === 'tool' ? text('Tool activity', '工具进展') : kindLabel(message.kind)}>
              {message.role === 'assistant' && message.kind && !['reply', 'plan'].includes(message.kind) && <div className={styles.messageLabel}>{kindLabel(message.kind)}</div>}
              {message.role === 'assistant' && message.metadata?.collaboration && <CollaborationCard collaboration={message.metadata.collaboration} />}
              {message.role === 'assistant' && message.metadata?.card && <InlineBusinessCard card={message.metadata.card} />}
              {message.role === 'assistant' && message.metadata?.plan
                ? <PlanCard plan={message.metadata.plan} text={message.text} locale={locale} />
                : message.role === 'assistant' && message.metadata?.choices?.length
                ? <>
                  <ReplyText id={`${message.id}-prompt`} text="请选择一个回答，也可以输入自定义内容。" locale={locale} />
                  <QuestionCard message={message}
                  answer={messages.find(item => item.role === 'user' && item.seq > message.seq)?.text}
                  disabled={Boolean(archived || conversation.active || conversation.sending || conversation.pending || conversation.replyTo || conversation.uploading || conversation.sessionBusy || conversation.draft.trim() || conversation.attachments.length || messages.some(item => item.role === 'user' && item.seq > message.seq))}
                  onAnswer={answer => { void conversation.answerQuestion(message.id, answer) }} />
                </>
                : message.role === 'assistant'
                ? <ReplyText id={message.id} text={message.text} locale={locale}
                    failureReason={message.kind === 'failed' ? turnsById.get(message.turn_id)?.failure_reason || message.text : undefined} />
                : message.role === 'tool' ? <ToolActivityGroup messages={group} onSelectModel={conversation.setModel} now={conversation.snapshot?.server_time} active={turnsById.get(message.turn_id)?.state === 'running'}
                  runningId={conversation.active && !conversation.error ? currentTool?.id : undefined}
                  awaiting={approvals.filter(approval => approval.turn_id === message.turn_id).map(approval => approval.tool_name)} /> : <>
                    {(turnsById.get(message.turn_id)?.reply_to_message_id || turnsById.get(message.turn_id)?.reply_quote) && <ReplyQuote locale={locale}
                      quote={turnsById.get(message.turn_id)?.reply_quote}
                      message={messages.find(item => item.id === turnsById.get(message.turn_id)?.reply_to_message_id)} />}
                    <p>{message.text}</p>
                    {turnsById.get(message.turn_id)?.kind === 'queue' && <small>已取消 · 未执行</small>}
                    {unreportedTurns.filter(turn => turn.turn_id === message.turn_id).map(turn => <TurnNotice key={turn.turn_id} turn={turn} locale={locale} />)}
                  </>}
              {(message.attachments?.length ?? 0) > 0 && <ul className={styles.messageAttachments}>
                {message.attachments?.map(attachment => {
                  const href = conversation.snapshot?.conversation_id ? conversationAttachmentUrl(conversation.snapshot.conversation_id, attachment.id) : '#'
                  return <li key={attachment.id}>
                    {attachment.mime_type.startsWith('image/') ? <a className={styles.messageImageAttachment} href={href} target="_blank" rel="noreferrer"><img src={href} alt={attachment.name} loading="lazy" /></a> : <FileText size={16} aria-hidden="true" />}
                    <a className={styles.attachmentLink} href={href} target="_blank" rel="noreferrer"><span>{attachment.name}</span><small>{fileSize(attachment.size)}</small></a>
                    <a className={styles.attachmentDownload} href={href} download={attachment.name} aria-label={`下载 ${attachment.name}`}><Download size={14} /></a>
                  </li>
                })}
              </ul>}
              {message.role === 'assistant' && <ReplyMeta message={message} locale={locale}
                disabled={controlsDisabled || Boolean(conversation.pending)} regenerateDisabled={regenerateDisabled}
                regenerate={message.id === latestReply?.id ? () => { void conversation.send(message.id) } : undefined}
                fork={typeof conversation.forkConversation === 'function' && lastReplies.get(message.turn_id) === message.id
                  && turnsById.has(message.turn_id) && !['running', 'queued'].includes(turnsById.get(message.turn_id)!.state)
                  ? () => { void conversation.forkConversation(message.id).catch(() => {}) } : undefined}
                forkDisabled={Boolean(conversation.sessionBusy || conversation.sending || conversation.uploading || conversation.approvalBusy || conversation.shareBusy || conversation.pending)}
                onFollowup={prompt => {
                  quoteReply(message.id, selectionQuote?.messageId === message.id ? selectionQuote.text : '')
                  if (prompt) conversation.setDraft(`${conversation.draft ? `${conversation.draft}\n` : ''}${prompt}`.slice(0, 8000))
                  draftInput.current?.focus()
                }}
                turn={lastReplies.get(message.turn_id) === message.id ? turnsById.get(message.turn_id) : undefined} />}
            </article>})}
            {unreportedTurns.filter(turn => !messages.some(message => message.turn_id === turn.turn_id && message.role === 'user'))
              .map(turn => <TurnNotice key={turn.turn_id} turn={turn} locale={locale} />)}
            {(conversation.snapshot?.approvals?.length ?? 0) > 0 && <section className={styles.actionSequence} aria-label="动作流程">
              <header><ShieldCheck size={14} /><strong>动作流程</strong><span>{approvals.length ? `${approvals.length} 项待确认` : '确认记录'}</span></header>
            {(conversation.snapshot?.approvals ?? []).map(approval => <section key={approval.approval_id} className={styles.approval}
              data-state={approval.state} aria-busy={conversation.approvalBusy === approval.approval_id}
              aria-label={text('Action approval', '动作确认')} data-approval-id={approval.approval_id}>
              <div className={styles.approvalTitle}><ShieldCheck size={16} aria-hidden="true" />
                <strong>{approvalAction(approval.tool_name)}</strong><span>{approval.state === 'pending' ? text('Awaiting approval', '等待确认')
                  : approval.state === 'approved' ? text('Allowed', '已允许') : approval.state === 'denied' ? text('Denied', '已拒绝') : text('Closed', '已关闭')}</span></div>
              {approval.tool_name === 'run_python' && <><p>将在 Host 运行 Python 分析，使用本轮读取的观测数据。此环境不是隔离沙箱。</p><pre className={styles.approvalArguments}>{String(approval.arguments.code ?? '')}</pre></>}
              {approval.tool_name === 'save_map_copy' && <dl className={styles.approvalFields}>
                <div><dt>原地图</dt><dd>{String(approval.arguments.map_id ?? '—')}</dd></div>
                <div><dt>另存为</dt><dd>{String(approval.arguments.name ?? '—')}</dd></div>
                <div><dt>修改</dt><dd>{Array.isArray(approval.arguments.operations) ? `${approval.arguments.operations.length} 项` : '—'}</dd></div>
              </dl>}
              {approval.tool_name !== 'run_python' && Object.keys(approval.arguments).length > 0 && <details className={styles.approvalParameters}><summary>操作参数</summary>
                <p className={styles.approvalArguments}>{approvalArguments(approval.arguments)}</p></details>}
              {approval.state === 'pending' && <div className={styles.approvalActions}>
                <WorkTime turn={turnsById.get(approval.turn_id)} serverTime={conversation.snapshot?.server_time} locale={locale} />
                <button type="button" disabled={conversation.approvalBusy === approval.approval_id || Boolean(conversation.sessionBusy)}
                  onClick={() => { void conversation.decideApproval(approval.approval_id, false) }}>{text('Reject', '拒绝')}</button>
                <button type="button" className={styles.approveButton} aria-busy={conversation.approvalBusy === approval.approval_id} disabled={conversation.approvalBusy === approval.approval_id || Boolean(conversation.sessionBusy)}
                  onClick={() => { void conversation.decideApproval(approval.approval_id, true) }}>{text('Allow action', '允许执行')}</button>
              </div>}
            </section>)}
            </section>}
            <div ref={historyEnd} />
          </div>
          {showLatest && <button type="button" className={styles.latest} onClick={() => {
            followLatest.current = true
            historyEnd.current?.scrollIntoView({ block: 'nearest' })
            setAwayConversation(null)
          }}>{text('Back to latest', '回到最新')}<ArrowUp size={13} aria-hidden="true" /></button>}
          {conversation.notice && <p className={styles.notice} role="status">{conversation.notice}</p>}
        </div>
      )}
      {!conversation.sessionListOpen && (workStatus || !conversation.options) && <div className={styles.botStatus} role="status"
        data-tone={botState === 'waiting' ? 'waiting' : botState === 'error' || botState === 'unavailable' ? 'error' : 'working'}>
        <div className={styles.botCaption}><span className={styles.workLabel} data-working={['processing', 'working', 'writing'].includes(botState)}
          title={conversation.optionsError || undefined}>{botLabel}</span>
          {conversation.active && !approvals.length && <WorkTime turn={turns.find(turn => turn.state === 'running')}
            serverTime={conversation.snapshot?.server_time} locale={locale} />}
        </div>
        {(conversation.optionsError || conversation.error) && <button type="button" className={styles.iconButton}
          onClick={() => { void conversation.reconnect() }} disabled={conversation.optionsLoading}
          aria-label={text('Refresh model connection', '刷新模型连接')}><RotateCw size={16} /></button>}
      </div>}
      {approvals.length > 0 && <nav className={styles.pendingActions} aria-label="待确认动作">
        <span>{approvals.length} 项待确认</span>
        {approvals.map(approval => <button type="button" key={approval.approval_id} onClick={() => {
          conversation.setSessionListOpen(false)
          conversation.setOpen(true)
          requestAnimationFrame(() => {
            const target = [...(root.current?.querySelectorAll<HTMLElement>('[data-approval-id]') ?? [])]
              .find(element => element.dataset.approvalId === approval.approval_id)
            target?.scrollIntoView({ block: 'nearest', behavior: window.matchMedia('(prefers-reduced-motion: reduce)').matches ? 'instant' : 'smooth' })
            target?.querySelector<HTMLButtonElement>('button:not(:disabled)')?.focus({ preventScroll: true })
          })
        }}><ShieldCheck size={12} />{approvalAction(approval.tool_name)}<ChevronDown size={12} /></button>)}
      </nav>}
      {conversation.error && !conversation.optionsError && <p className={styles.connectionError} role="status">{conversation.error}</p>}
      {!messages.length && !conversation.active && !conversation.sessionListOpen && conversation.options && !archived && <div className={styles.suggestions}>
        {[text('Check robot status', '查看机器人状态'), mapSettings?.map
          ? text('Inspect this map', '检查当前地图') : text('What can you help with?', '你能帮我做什么')].map(prompt => <button
          key={prompt} type="button" disabled={controlsDisabled} onClick={() => {
            conversation.setDraft(prompt)
            draftInput.current?.focus()
          }}>{prompt}</button>)}
      </div>}
      {conversation.context && <div className={styles.contextBar} aria-label={text('Context for this message', '本次消息的上下文')}>
        {conversation.includeMapContext ? <>
          <div className={styles.contextChip}>
            <details><summary><MapIcon size={13} aria-hidden="true" /><span>{conversation.context.map_id}</span></summary>
              <div className={styles.contextDetails}><strong>{text('Map', '地图')} · {conversation.context.map_id}</strong>
                <span>{text('Content version', '内容版本')} · {conversation.context.content_epoch}</span></div>
            </details>
            <button type="button" disabled={Boolean(conversation.pending) || !conversation.editable}
              aria-label={text('Remove map context', '移除地图上下文')} onClick={() => conversation.setIncludeMapContext(false)}><X size={12} aria-hidden="true" /></button>
          </div>
          {conversation.context.selection && (conversation.includeSelection ? <div className={styles.contextChip}>
            <details><summary><Scan size={13} aria-hidden="true" /><span>{text('Selected area', '框选范围')}</span></summary>
              <div className={styles.contextDetails}><strong>{text('3D bounds', '三维范围')}</strong>
                <span>Min · {conversation.context.selection.min.map(value => value.toFixed(2)).join(', ')}</span>
                <span>Max · {conversation.context.selection.max.map(value => value.toFixed(2)).join(', ')}</span></div>
            </details>
            <button type="button" disabled={Boolean(conversation.pending) || !conversation.editable}
              aria-label={text('Remove selected area context', '移除框选上下文')} onClick={() => conversation.setIncludeSelection(false)}><X size={12} aria-hidden="true" /></button>
          </div> : <button type="button" className={styles.contextAdd} disabled={Boolean(conversation.pending) || !conversation.editable}
            onClick={() => conversation.setIncludeSelection(true)}><Plus size={12} aria-hidden="true" />{text('Attach selected area', '附加框选范围')}</button>)}
        </> : <button type="button" className={styles.contextAdd} disabled={Boolean(conversation.pending) || !conversation.editable}
          onClick={() => conversation.setIncludeMapContext(true)}><Plus size={12} aria-hidden="true" />{text('Attach current map', '附加当前地图')}</button>}
        {mapSettings?.agentPreview && <button type="button" className={styles.contextAdd} onClick={() => mapSettings.setAgentPreview(null)}>退出预览</button>}
      </div>}
      <QueuedMessages key={conversation.snapshot?.conversation_id} turns={turns} messages={messages}
        editing={conversation.queuedEdit ?? null} setEditing={conversation.setQueuedEdit}
        onChange={conversation.changeQueuedMessage} onRecover={conversation.setDraft}
        draftEmpty={!conversation.draft && !conversation.pending && !conversation.attachments.length && !conversation.replyTo} />
      <form className={styles.composer} onSubmit={event => { event.preventDefault(); void conversation.send() }}>
        {conversation.replyTo && <ReplyQuote locale={locale} message={messages.find(message => message.id === conversation.replyTo)}
          quote={conversation.replyQuote}
          disabled={Boolean(conversation.pending) || !conversation.editable} onClear={() => conversation.setReplyTo('')} />}
        <div className={styles.draftArea}>
        <textarea ref={draftInput} value={conversation.draft} onChange={event => conversation.setDraft(event.target.value)} rows={1}
          onFocus={() => { if (conversation.options) conversation.setOpen(true) }} readOnly={!conversation.editable || archived} disabled={conversation.sending || Boolean(conversation.sessionBusy)}
          onKeyDown={event => {
            if (event.nativeEvent.isComposing || event.keyCode === 229) {
              if (event.key === 'Enter') event.preventDefault()
              return
            }
            if (event.key === 'Enter' && !event.shiftKey) { event.preventDefault(); void conversation.send() }
            if (event.key === 'Escape') { conversation.setOpen(false); event.currentTarget.blur() }
          }}
          placeholder={mapSettings?.map ? text('Edit this map or find an inspection point…', '补齐选区、标注玻璃、查找巡检点…') : text('Talk to the robot…', '与机器人对话…')} aria-label={text('Message to the robot', '与机器人对话')}
          autoComplete="off" maxLength={8000} />
        {conversation.attachments.length > 0 && <ul className={styles.attachments} aria-label={text('Message attachments', '待发送附件')}>
          {conversation.attachments.map(attachment => <li key={attachment.id}>
            <Paperclip size={14} aria-hidden="true" /><span className={styles.attachmentName} title={attachment.name}>{attachment.name}</span>
            <span className={styles.fileSize}>{fileSize(attachment.size)}</span>
            <button type="button" className={styles.iconButton} disabled={controlsDisabled}
              onClick={() => conversation.removeAttachment(attachment.id)} aria-label={text(`Remove ${attachment.name}`, `移除 ${attachment.name}`)}><X size={14} /></button>
          </li>)}
        </ul>}
        </div>
        <div className={styles.toolbar}>
          <input ref={fileInput} type="file" className={styles.fileInput} multiple accept={model?.attachment_accept ?? conversation.options?.attachments.accept}
            disabled={controlsDisabled} tabIndex={-1} aria-hidden="true" onChange={event => {
              const files = Array.from(event.target.files ?? [])
              event.target.value = ''
              void conversation.addAttachments(files)
            }} />
          <ComposerAdd locale={locale} disabled={controlsDisabled} hasMap={Boolean(mapSettings?.map)}
            onFiles={() => fileInput.current?.click()}
            onPrompt={prompt => { conversation.setDraft(`${conversation.draft ? `${conversation.draft}\n` : ''}${prompt}`.slice(0, 8000)); draftInput.current?.focus() }} />
          <ComposerMenu label={text('Permissions', '权限')} value={conversation.permissionMode ?? 'default'} options={[
            { id: 'read_only', label: text('Read only', '只读') },
            { id: 'default', label: text('Ask before actions', '执行前确认') },
            { id: 'full_access', label: 'Full access' },
          ]} disabled={controlsDisabled} permission onChange={value => conversation.setPermissionMode(value as typeof conversation.permissionMode)} />
          {conversation.uploading && <span className={styles.uploading} role="status">{text('Uploading…', '上传中…')}</span>}
          <button type="button" className={styles.iconButton} aria-label="管理模型连接" title="管理模型连接"
            onClick={() => setConnectionsOpen(true)}><SlidersHorizontal size={15} /></button>
          {conversation.options && <div className={styles.modelControls}>
            <ComposerMenu label={text('Model', '模型')} value={conversation.model} options={conversation.options?.models ?? []}
              disabled={controlsDisabled} placeholder={conversation.optionsLoading ? text('Connecting…', '连接中…') : text('Disconnected', '未连接')}
              onChange={conversation.setModel} />
            <ComposerMenu label={text('Thinking depth', '思考深度')} value={conversation.reasoning} options={model?.reasoning ?? []}
              disabled={controlsDisabled || (model?.reasoning.length ?? 0) < 2} reasoning onChange={conversation.setReasoning} />
          </div>}
          <button type="submit" className={styles.sendButton} disabled={!conversation.options || model?.configured === false || conversation.sending || conversation.uploading || archived || Boolean(conversation.sessionBusy)
            || (!conversation.draft.trim() && !conversation.attachments.length && !conversation.pending)}
            aria-label={conversation.pending ? text('Retry original message', '重试原消息') : conversation.active ? text('Queue message', '加入待执行') : text('Send message', '发送消息')}>
            {conversation.pending ? <RotateCw size={17} /> : <ArrowUp size={19} />}</button>
        </div>
      </form>
      {sharingId && sharingId === currentId && <ConversationShareCard key={sharingId} locale={locale}
        title={conversation.snapshot?.metadata?.title ?? conversation.conversations?.find(session => session.conversation_id === currentId)?.title ?? ''}
        snapshot={conversation.snapshot} url={conversation.shareUrl} busy={conversation.shareBusy}
        local={typeof location !== 'undefined' && ['localhost', '127.0.0.1'].includes(location.hostname)}
        onCreate={conversation.shareCurrentConversation} onRevoke={conversation.revokeShare} onClose={() => setSharingId(null)} />}
      {connectionsOpen && <ModelConnections options={conversation.options} onClose={() => setConnectionsOpen(false)}
        onChanged={() => { void conversation.reconnect() }} />}
      {botStudioOpen && <BotStudio profile={botProfile} state={botState} permission={conversation.permissionMode}
        permissionsAvailable={Boolean(conversation.options)} locked={Boolean(conversation.active || conversation.sending || conversation.pending || conversation.sessionBusy)}
        onClose={() => setBotStudioOpen(false)} onSave={(profile, permission) => {
          const saved = saveBotProfile(profile)
          setBotProfile(saved)
          if (conversation.options && permission !== conversation.permissionMode) conversation.setPermissionMode(permission)
          return true
        }} />}
    </section>
  )
}
