import { useEffect, useRef, useState } from 'react'
import type { PointerEvent } from 'react'
import { ArrowUpRight, Copy, Eye, LoaderCircle, RotateCw, X } from 'lucide-react'
import type { Locale } from '../i18n'
import { renderMarkdown } from '../guide/markdown'
import { fetchSharedConversation, type ConversationSnapshot, type SharedConversationSnapshot } from '../services/agentConversation'
import { ModalShell } from './Modal'
import { PixelCheck, SuccessCheck } from './SuccessCheck'
import styles from './ConversationShareCard.module.css'

interface Props {
  locale: Locale
  title: string
  snapshot?: ConversationSnapshot | null
  url: string
  busy: boolean
  local: boolean
  onCreate(): Promise<string | void>
  onRevoke(): Promise<void>
  onClose(): void
}

function resetFoil(card: HTMLDialogElement) {
  card.removeAttribute('data-foil-active')
  card.style.removeProperty('--foil-x')
  card.style.removeProperty('--foil-y')
}

function cleanTitle(value: string) {
  return value
    .replace(/&#x20;|&#32;|&nbsp;/gi, ' ')
    .replace(/^\s*#{1,6}\s*/, '')
    .replace(/\s+/g, ' ')
    .trim()
}

export function ConversationShareCard({ locale, title, snapshot, url, busy, local, onCreate, onRevoke, onClose }: Props) {
  const input = useRef<HTMLInputElement>(null)
  const [copyState, setCopyState] = useState<{ mode: 'copied' | 'manual'; sequence: number } | null>(null)
  const [action, setAction] = useState<'creating' | 'revoking' | 'copying' | null>(null)
  const [requestError, setRequestError] = useState('')
  const [manualUrl, setManualUrl] = useState('')
  const [attempt, setAttempt] = useState(0)
  const [result, setResult] = useState<{ token: string; snapshot?: SharedConversationSnapshot; error?: boolean } | null>(null)
  const text = (en: string, zh: string) => locale === 'zh' ? zh : en
  const token = snapshot?.share?.token
  const current = token && result?.token === token ? result : null
  const loading = Boolean(token && !current)
  const terminalTurns = new Set(snapshot?.turns.filter(turn => !['running', 'queued'].includes(turn.state)).map(turn => turn.turn_id))
  const messages = token ? current?.snapshot?.messages ?? [] : snapshot?.messages.flatMap(message =>
    message.role !== 'tool' && terminalTurns.has(message.turn_id)
      ? [{ role: message.role, text: message.text, seq: message.seq }] : []) ?? []
  const sharedTitle = cleanTitle(current?.snapshot?.title ?? title)
  const operation = action ?? (busy ? url ? 'revoking' : 'creating' : null)
  const pending = Boolean(operation || loading)
  const operationText = {
    creating: text('Generating…', '正在生成…'),
    revoking: text('Revoking…', '正在撤销…'),
    copying: text('Copying…', '正在复制…'),
  }
  const buttonLabel = operation ? operationText[operation]
    : loading ? text('Loading…', '读取中…') : copyState?.mode === 'copied' ? text('Copied', '已复制')
      : url ? text('Copy link', '复制链接') : text('Generate link', '生成链接')
  const snapshotNote = text('Messages added after sharing will stay outside this snapshot.', '分享后新增的消息不会加入这份快照。')

  function moveFoil(event: PointerEvent<HTMLDialogElement>) {
    const card = event.currentTarget
    if (event.pointerType !== 'mouse' || window.matchMedia('(prefers-reduced-motion: reduce)').matches) {
      resetFoil(card)
      return
    }
    const bounds = card.getBoundingClientRect()
    const x = (event.clientX - bounds.left) / bounds.width
    const y = (event.clientY - bounds.top) / bounds.height
    if (x < 0 || x > 1 || y < 0 || y > 1) {
      resetFoil(card)
      return
    }
    card.style.setProperty('--foil-x', `${Math.round(x * 100)}%`)
    card.style.setProperty('--foil-y', `${Math.round(y * 100)}%`)
    card.setAttribute('data-foil-active', '')
  }

  useEffect(() => {
    if (!token) return
    let disposed = false
    fetchSharedConversation(token).then(value => {
      if (!disposed) setResult({ token, snapshot: value })
    }).catch(() => {
      if (!disposed) setResult({ token, error: true })
    })
    return () => { disposed = true }
  }, [token, attempt])

  async function copyLink(targetUrl = url) {
    setRequestError('')
    setManualUrl(targetUrl)
    setAction('copying')
    try {
      await navigator.clipboard.writeText(targetUrl)
      setCopyState(current => ({ mode: 'copied', sequence: (current?.sequence ?? 0) + 1 }))
    } catch {
      if (copyState?.mode === 'manual') { input.current?.focus(); input.current?.select() }
      setCopyState(current => ({ mode: 'manual', sequence: current?.sequence ?? 0 }))
    } finally {
      setAction(null)
    }
  }

  useEffect(() => {
    if (copyState?.mode === 'manual') { input.current?.focus(); input.current?.select() }
  }, [copyState?.mode])

  return <ModalShell open title={sharedTitle || text('Conversation', '对话')}
    className={styles.dialog} onClose={onClose}
    onPointerMove={moveFoil} onPointerLeave={event => resetFoil(event.currentTarget)}
    headerAction={<button type="button" data-modal-cancel className={styles.close}
      aria-label={text('Close sharing', '关闭分享')} onClick={onClose}><X size={18} aria-hidden="true" /></button>}
    footer={<div className={styles.footer}>
      <p className={styles.access} title={snapshotNote}>
        <Eye size={15} aria-hidden="true" />
        <span>{local ? text('Read-only on this device only', '本机只读 · 仅此设备可打开')
          : text('Anyone with the link can view this snapshot', '持链接且能访问服务者可查看')}</span>
      </p>
      {url && <a className={styles.viewLink} href={url} target="_blank" rel="noopener noreferrer">
        {text('View conversation', '查看会话')}<ArrowUpRight size={15} aria-hidden="true" />
      </a>}
      {url && <button type="button" className={styles.revoke} disabled={Boolean(busy || action)} onClick={async () => {
        setRequestError('')
        setCopyState(null)
        setAction('revoking')
        try { await onRevoke() }
        catch (failure) { setRequestError(failure instanceof Error ? failure.message : text('Unable to revoke the link.', '撤销链接失败。')) }
        finally { setAction(null) }
      }}>{operation === 'revoking' ? operationText.revoking : text('Revoke link', '撤销链接')}</button>}
      <button type="button" className={styles.primary} aria-label={buttonLabel} title={buttonLabel}
        data-pending={pending || undefined} disabled={pending || Boolean(current?.error) || !messages.length}
        onClick={url ? () => copyLink() : async () => {
          setRequestError('')
          setCopyState(null)
          setAction('creating')
          try {
            const createdUrl = await onCreate()
            if (createdUrl) await copyLink(createdUrl)
          }
          catch (failure) { setRequestError(failure instanceof Error ? failure.message : text('Unable to generate the link.', '生成链接失败。')) }
          finally { setAction(null) }
        }}>
        {pending ? <LoaderCircle className={styles.spinner} size={20} aria-hidden="true" />
          : copyState?.mode === 'copied' ? <PixelCheck key={copyState.sequence} className={styles.check} size={22} />
            : url ? <Copy size={19} aria-hidden="true" /> : <svg className={styles.shareGlyph} viewBox="0 0 24 24" fill="none"
              stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true" data-share-glyph="">
              <path d="M8.1 10.9 15.9 6.8M8.1 13.1l7.8 4.1" />
              <circle cx="6.1" cy="12" r="2.5" fill="currentColor" stroke="none" />
              <circle cx="17.9" cy="6" r="2.5" fill="currentColor" stroke="none" />
              <circle cx="17.9" cy="18" r="2.5" fill="currentColor" stroke="none" />
            </svg>}
        <span role="status" className={styles.buttonText}>{buttonLabel}</span>
      </button>
    </div>}>
    <div className={styles.material} aria-hidden="true">
      <svg className={styles.wordmark} viewBox="0 0 272 50" fill="currentColor" aria-label="LINGTU">
        <path d="M2 3H9V39H36V46H2Z" />
        <path d="M48 3H55V46H48Z" />
        <path d="M67 46V3H74L100 33V3H107V46H100L74 16V46Z" />
        <path d="M127 3H160V10H130L126 14V35L130 39H153V29H141V22H160V46H127L119 38V11Z" />
        <path d="M172 3H216V10H198V46H190V10H172Z" />
        <path d="M228 3H235V34L240 39H258L263 34V3H270V37L261 46H237L228 37Z" />
      </svg>
    </div>
    <p className={styles.subtitle}>{!token && snapshot?.history_cursor != null
      ? text('This preview shows recent messages. The link will include all finished text turns.', '此处预览最近的消息，链接将包含全部已结束的文字对话。')
      : snapshotNote}</p>
    <div className={styles.previewFrame}>
      <div className={styles.preview} role="region" tabIndex={0}
        aria-label={text('Shared conversation preview', '分享内容预览') + (sharedTitle ? `: ${sharedTitle}` : '')} aria-busy={loading}>
        {loading && <div className={styles.previewStatus} role="status">
          <LoaderCircle className={styles.spinner} size={20} aria-hidden="true" />
          <p>{text('Loading shared conversation…', '正在读取分享内容…')}</p>
        </div>}
        {current?.error && <div className={styles.previewStatus} role="alert">
          <p>{text('Unable to load this shared snapshot.', '分享快照读取失败。')}</p>
          <button type="button" className={styles.retry} onClick={() => { setResult(null); setAttempt(value => value + 1) }}>
            <RotateCw size={14} aria-hidden="true" />{text('Retry', '重试')}</button>
        </div>}
        {!loading && !current?.error && !messages.length && <p className={styles.previewStatus}>
          {text('This conversation has no finished text to share yet.', '此会话还没有可分享的已结束文字对话。')}</p>}
        {messages.map(message => <article key={message.seq}
          className={`${styles.message} ${message.role === 'user' ? styles.user : styles.assistant}`}>
          {message.role === 'assistant' ? <div className={styles.richText} dangerouslySetInnerHTML={{ __html: renderMarkdown(message.text, {
            pageId: 'share-preview', resolveDocumentLink: () => undefined, renderImages: true,
          }) }} /> : <p>{message.text}</p>}
        </article>)}
      </div>
      {copyState?.mode === 'copied' && !pending && <div key={copyState.sequence} className={styles.successOverlay} aria-hidden="true" data-copy-success={copyState.sequence}>
        <SuccessCheck />
      </div>}
    </div>
    {copyState?.mode === 'manual' && <div className={styles.manualCopy}>
      <input ref={input} readOnly value={manualUrl || url} aria-label={text('Share link', '分享链接')} onFocus={event => event.currentTarget.select()} />
      <p role="status">{text('Copy the selected link.', '请复制已选中的链接。')}</p>
    </div>}
    {requestError && <p className={styles.error} role="alert">{requestError}</p>}
  </ModalShell>
}
