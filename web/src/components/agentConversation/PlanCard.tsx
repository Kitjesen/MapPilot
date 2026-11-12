import { useEffect, useRef, useState } from 'react'
import { ChevronDown, Copy, Download } from 'lucide-react'
import type { Locale } from '../../i18n'
import type { ConversationPlan } from '../../services/agentConversation'
import styles from './PlanCard.module.css'

export function PlanCard({ plan, text, locale }: { plan: ConversationPlan; text: string; locale: Locale }) {
  const original = useRef<HTMLTextAreaElement>(null)
  const [copyState, setCopyState] = useState<'copying' | 'copied' | 'failed' | null>(null)
  const [downloadFailed, setDownloadFailed] = useState(false)
  const label = (en: string, zh: string) => locale === 'zh' ? zh : en

  useEffect(() => {
    if (copyState === 'failed') { original.current?.focus(); original.current?.select() }
  }, [copyState])

  async function copy() {
    setCopyState('copying')
    try { await navigator.clipboard.writeText(text); setCopyState('copied') }
    catch { setCopyState('failed'); original.current?.focus(); original.current?.select() }
  }

  function download() {
    setDownloadFailed(false)
    let url = ''
    try {
      url = URL.createObjectURL(new Blob([text], { type: 'text/markdown;charset=utf-8' }))
      const link = document.createElement('a')
      link.href = url
      link.download = 'lingtu-plan.md'
      link.click()
    } catch { setDownloadFailed(true) }
    finally { if (url) setTimeout(() => URL.revokeObjectURL(url), 1000) }
  }

  return <section className={styles.card} aria-label={label('Plan draft', '方案草稿')}>
    <details className={styles.details}>
      <summary aria-label={label('Plan: ', '方案：') + plan.title}>
        <span className={styles.draft}>{label('Draft · Not executed', '方案草稿 · 未执行')}</span>
        <strong className={styles.title}>{plan.title}</strong>
        {plan.summary && <span className={styles.description}>{plan.summary}</span>}
        <span className={styles.toggle}>{label(`${plan.steps.length} steps`, `${plan.steps.length} 个步骤`)}<ChevronDown size={14} aria-hidden="true" /></span>
      </summary>
      <ol className={styles.steps}>{plan.steps.map((step, index) => <li key={index}>
        <strong>{step.title}</strong>
        {step.detail && <p>{step.detail}</p>}
      </li>)}</ol>
    </details>
    <footer className={styles.actions}>
      <button type="button" disabled={copyState === 'copying'} onClick={() => { void copy() }}
        aria-label={label('Copy plan Markdown', '复制方案 Markdown')}>
        <Copy size={14} aria-hidden="true" />
        <span role="status">{copyState === 'copying' ? label('Copying…', '正在复制…')
          : copyState === 'copied' ? label('Copied', '已复制') : label('Copy', '复制')}</span>
      </button>
      <button type="button" onClick={download} aria-label={label('Download plan Markdown', '下载方案 Markdown')}>
        <Download size={14} aria-hidden="true" />{label('Download .md', '下载 .md')}
      </button>
    </footer>
    {copyState === 'failed' && <div className={styles.fallback}>
      <p role="alert">{label('Copy failed. Select the original text below to copy it.', '复制失败，请选中下方原文复制。')}</p>
      <textarea ref={original} readOnly value={text} rows={5} aria-label={label('Plan Markdown source', '方案 Markdown 原文')}
        onFocus={event => event.currentTarget.select()} />
    </div>}
    {downloadFailed && <p className={styles.error} role="alert">{label('Download failed. Copy the plan to save it as Markdown.', '下载失败，可复制方案后保存为 Markdown。')}</p>}
  </section>
}
