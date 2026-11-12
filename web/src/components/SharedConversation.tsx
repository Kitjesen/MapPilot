import { useEffect, useRef, useState } from 'react'
import { ArrowDownToLine, Link2, RotateCw } from 'lucide-react'
import { renderMarkdown } from '../guide/markdown'
import type { Locale } from '../i18n'
import {
  ConversationError, fetchSharedConversation, type SharedConversationSnapshot,
} from '../services/agentConversation'
import styles from './SharedConversation.module.css'

interface SharedConversationProps {
  token: string
  locale: Locale
}

export function SharedConversation({ token, locale }: SharedConversationProps) {
  const pageRef = useRef<HTMLElement>(null)
  const [preparingPdf, setPreparingPdf] = useState(false)
  const [missingImages, setMissingImages] = useState(false)
  const [attempt, setAttempt] = useState(0)
  const [result, setResult] = useState<{
    token: string
    snapshot?: SharedConversationSnapshot
    error?: Error
  } | null>(null)
  const text = (en: string, zh: string) => locale === 'zh' ? zh : en
  const current = result?.token === token ? result : null

  useEffect(() => {
    let disposed = false
    fetchSharedConversation(token).then(snapshot => {
      if (!disposed) setResult({ token, snapshot })
    }).catch((error: Error) => {
      if (!disposed) setResult({ token, error })
    })
    return () => { disposed = true }
  }, [token, attempt])

  const error = current?.error
  const snapshot = current?.snapshot
  const title = snapshot?.title
  useEffect(() => {
    if (!title) return
    const previousTitle = document.title
    document.title = `${title} · LingTu`
    return () => { document.title = previousTitle }
  }, [title])
  const unavailable = error instanceof ConversationError && error.status === 404
  const errorText = unavailable ? text('This share has been revoked or does not exist.', '分享已撤销或不存在。')
    : error instanceof ConversationError && error.status === 503
      ? text('The sharing service is unavailable.', '分享服务暂不可用。')
      : text('Unable to load this conversation.', '无法读取这份对话。')

  async function savePdf() {
    setPreparingPdf(true)
    setMissingImages(false)
    try {
      const images = Array.from(pageRef.current?.querySelectorAll('img') ?? [])
      const loaded = await Promise.all(images.map(image => {
        image.loading = 'eager'
        return image.decode().then(() => true, () => false)
      }))
      if (loaded.some(success => !success)) setMissingImages(true)
      else window.print()
    } finally {
      setPreparingPdf(false)
    }
  }

  return <main ref={pageRef} className={styles.page}>
    <div className={styles.content}>
      <div className={styles.topbar}>
        <span className={styles.wordmark}>LingTu<span className={styles.wordmarkDot}>.</span></span>
        {snapshot && <button type="button" className={styles.pdfButton} disabled={preparingPdf}
          onClick={() => void savePdf()} aria-describedby="shared-pdf-hint">
          <ArrowDownToLine size={16} aria-hidden="true" />
          {preparingPdf ? text('Preparing…', '正在准备…') : text('Save as PDF', '保存为 PDF')}
        </button>}
      </div>
      <header className={styles.header}>
        <span className={styles.badge}><Link2 size={14} aria-hidden="true" />{text('Read-only share', '只读分享')}</span>
        {snapshot && <>
          <h1>{snapshot.title}</h1>
          <time dateTime={new Date(snapshot.created_at * 1000).toISOString()}>
            {new Intl.DateTimeFormat(locale === 'zh' ? 'zh-CN' : 'en', {
              dateStyle: 'medium', timeStyle: 'short',
            }).format(new Date(snapshot.created_at * 1000))}
          </time>
          <p className={styles.snapshotNote}>{text('A snapshot of this conversation. New messages are not included.', '这份对话停留在分享时刻，后续消息不会加入。')}</p>
        </>}
      </header>
      {snapshot && <div className={styles.exportInfo}>
        <p id="shared-pdf-hint">{text('Choose “Save as PDF” in the print dialog.', '在打印窗口中选择“另存为 PDF”。')}</p>
        {missingImages && <div className={styles.pdfWarning} role="alert">
          <p>{text('Some images could not load. The PDF will be missing these images.', '部分图片加载失败，保存的 PDF 会缺少这些图片。')}</p>
          <button type="button" onClick={() => window.print()}>{text('Save without these images', '仍然保存')}</button>
        </div>}
      </div>}
      {!current && <p className={styles.status} role="status">{text('Loading conversation…', '正在读取分享…')}</p>}
      {error && <section className={styles.status} role="alert">
        <p>{errorText}</p>
        <button type="button" onClick={() => { setResult(null); setAttempt(value => value + 1) }}>
          <RotateCw size={15} aria-hidden="true" />{text('Retry', '重试')}
        </button>
      </section>}
      {snapshot && <div className={styles.messages} aria-label={text('Shared conversation', '分享对话')}>
        {snapshot.messages.map(message => <article key={message.seq}
          className={`${styles.message} ${message.role === 'user' ? styles.user : styles.assistant}`}>
          <div className={styles.role}>{message.role === 'user' ? text('You', '用户') : 'LingTu'}</div>
          {message.role === 'assistant'
            ? <div className={styles.richText} dangerouslySetInnerHTML={{ __html: renderMarkdown(message.text, {
              pageId: 'shared-conversation', resolveDocumentLink: () => undefined, renderImages: true,
            }) }} />
            : <p>{message.text}</p>}
        </article>)}
      </div>}
    </div>
  </main>
}
