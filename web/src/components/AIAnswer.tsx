import { useEffect, useMemo, useRef, useState } from 'react'
import { BookOpen, Check, ChevronDown, ChevronRight, Command, Copy, FileText, Globe2, MessageSquare, Moon, Palette, PanelLeft, RotateCw, Search, Sparkles, Sun, Tag, ThumbsDown, ThumbsUp, X } from 'lucide-react'
import styles from './AIAnswer.module.css'
import { answerBlocks, answerCopyText } from './aiAnswerModel'

export type AnswerSource = {
  name: string
  title: string
  href?: string
  icon?: string
  snippet?: string
}

export type AIAnswerProps = {
  question: string
  sources: AnswerSource[]
  answer: string
  streaming?: boolean
  animate?: boolean
  speed?: number
  followUps?: string[]
  onFollowUp?: (question: string) => void
  onRegenerate?: () => void
  onStop?: () => void
  onFeedback?: (value: 'up' | 'down', reason?: string) => void
  feedbackReasons?: string[]
  onCopy?: (text: string) => void
  framed?: boolean
  className?: string
}

const defaultReasons = ['Not accurate', 'Wrong source', 'Too long', 'Out of date']

const iconFor = (source: AnswerSource) => {
  if (source.icon) return <span className={styles.sourceIcon} style={{ background: source.icon }} aria-hidden="true">{source.name.slice(0, 1)}</span>
  if (/slack/i.test(source.name)) return <span className={styles.sourceIcon} style={{ background: '#4a154b' }} aria-hidden="true"><MessageSquare size={10} /></span>
  if (/wiki|web|browser/i.test(source.name)) return <span className={styles.sourceIcon} style={{ background: '#4a70a8' }} aria-hidden="true"><Globe2 size={10} /></span>
  return <span className={styles.sourceIcon} style={{ background: '#6a6d68' }} aria-hidden="true"><FileText size={10} /></span>
}

function renderInline(text: string, sources: AnswerSource[], activeCitation: number | null, onCitation: (index: number) => void) {
  const parts = text.split(/(\[\d+(?:,\s*\d+)*\]|\*\*[^*]+\*\*)/g).filter(Boolean)
  return parts.map((part, index) => {
    const citation = part.match(/^\[(\d+(?:,\s*\d+)*)\]$/)
    if (citation) {
      const numbers = citation[1].split(',').map(value => Number(value.trim())).filter(value => value >= 1 && value <= sources.length)
      if (!numbers.length) return null
      return <button key={`${part}-${index}`} type="button" className={styles.citation} data-active={activeCitation != null && numbers.includes(activeCitation)} aria-label={`Source: ${numbers.map(number => sources[number - 1].title).join(', ')}`} onClick={() => onCitation(numbers[0])}>{numbers.map(number => sources[number - 1].name).join(' + ')} {numbers.length > 1 ? `+${numbers.length - 1}` : ''}</button>
    }
    if (part.startsWith('**') && part.endsWith('**')) return <strong key={`${part}-${index}`}>{part.slice(2, -2)}</strong>
    return <span key={`${part}-${index}`}>{part}</span>
  })
}

function AnswerText({ answer, sources, activeCitation, onCitation, cursor }: { answer: string; sources: AnswerSource[]; activeCitation: number | null; onCitation: (index: number) => void; cursor: boolean }) {
  const blocks = answerBlocks(answer)
  return <div className={styles.answer} aria-live="polite">
    {blocks.map((block, index) => {
      const lines = block.split('\n').filter(Boolean)
      const list = lines.length > 0 && lines.every(line => /^-\s+/.test(line))
      if (list) return <ul key={index}>{lines.map(line => <li key={line}>{renderInline(line.replace(/^-\s+/, ''), sources, activeCitation, onCitation)}</li>)}</ul>
      return <p key={index}>{renderInline(block.replace(/\n/g, ' '), sources, activeCitation, onCitation)}{cursor && index === blocks.length - 1 && <span className={styles.answerCursor} aria-hidden="true" />}</p>
    })}
  </div>
}

export function AIAnswer({ question, sources, answer, streaming = false, animate = true, speed = 38, followUps = [], onFollowUp, onRegenerate, onStop, onFeedback, feedbackReasons = defaultReasons, onCopy, framed = false, className = '' }: AIAnswerProps) {
  const [visibleAnswer, setVisibleAnswer] = useState(animate ? '' : answer)
  const [activeCitation, setActiveCitation] = useState<number | null>(null)
  const [feedback, setFeedback] = useState<'up' | 'down' | null>(null)
  const [showReasons, setShowReasons] = useState(false)
  const [copyState, setCopyState] = useState<'idle' | 'done'>('idle')
  const answerRef = useRef(answer)

  useEffect(() => {
    if (!animate) return
    if (answerRef.current !== answer && answer.startsWith(answerRef.current)) {
      answerRef.current = answer
    } else {
      answerRef.current = answer
    }
    const words = answer.match(/\S+\s*/g) ?? []
    if (!words.length) return
    let index = 0
    const interval = window.setInterval(() => {
      index += 1
      setVisibleAnswer(words.slice(0, index).join(''))
      if (index >= words.length) window.clearInterval(interval)
    }, Math.max(18, Math.round(1000 / Math.max(1, speed))))
    return () => window.clearInterval(interval)
  }, [answer, animate, speed])

  useEffect(() => {
    const close = (event: KeyboardEvent) => { if (event.key === 'Escape') setActiveCitation(null) }
    window.addEventListener('keydown', close)
    return () => window.removeEventListener('keydown', close)
  }, [])

  const activeSource = activeCitation == null ? null : sources[activeCitation - 1]
  const sourceCopy = useMemo(() => answerCopyText(answer, sources), [answer, sources])
  const displayed = answer ? (animate ? visibleAnswer : answer) : ''
  const caughtUp = displayed.length >= answer.length

  const copy = async () => {
    try { await navigator.clipboard.writeText(sourceCopy); setCopyState('done'); onCopy?.(sourceCopy); window.setTimeout(() => setCopyState('idle'), 1600) }
    catch { onCopy?.(sourceCopy) }
  }

  return <section className={`${styles.aiAnswer} ${framed ? styles.framed : ''} ${className}`} aria-label="AI answer" aria-busy={streaming || !caughtUp}>
    <h3 className={styles.question}>{question}</h3>
    <div className={styles.sourceLabel}>Sources</div>
    <div className={styles.sources} aria-label="Sources">
      {sources.slice(0, 3).map((source, index) => <a key={`${source.name}-${source.title}`} className={styles.sourceCard} href={source.href ?? '#'} target={source.href ? '_blank' : undefined} rel={source.href ? 'noreferrer' : undefined} data-active={activeCitation === index + 1} onMouseEnter={() => setActiveCitation(index + 1)} onMouseLeave={() => setActiveCitation(null)} onClick={event => { if (!source.href) event.preventDefault(); setActiveCitation(index + 1) }}>
        <span className={styles.sourceTop}><span className={styles.sourceIdentity}>{iconFor(source)}{source.name}</span><span>{index + 1}</span></span>
        <span className={styles.sourceTitle}>{source.title}</span>
      </a>)}
      {sources.length > 3 && <button type="button" className={styles.sourceMore} data-active={activeCitation != null && activeCitation > 3} onClick={() => setActiveCitation(activeCitation != null && activeCitation > 3 ? null : 4)}><strong>+{sources.length - 3}</strong><span>more sources</span></button>}
    </div>
    {activeSource && <div className={styles.citationPreview} role="status"><strong>{activeSource.name} · {activeSource.title}</strong>{activeSource.snippet ?? 'Open the source to inspect the supporting context.'}<button type="button" className={styles.srOnly} onClick={() => setActiveCitation(null)} aria-label="Close source preview"><X size={12} /></button></div>}
    <div className={styles.status}>{streaming && <><Sparkles size={13} aria-hidden="true" />Writing</>}</div>
    <AnswerText answer={displayed} sources={sources} activeCitation={activeCitation} onCitation={setActiveCitation} cursor={streaming && caughtUp} />
    <div className={styles.actions}>
      {!streaming && <button type="button" onClick={() => void copy()} aria-label="Copy answer">{copyState === 'done' ? <Check size={14} /> : <Copy size={14} />}{copyState === 'done' ? 'Copied' : 'Copy'}</button>}
      {!streaming && onRegenerate && <button type="button" onClick={onRegenerate}><RotateCw size={14} />Regenerate</button>}
      {streaming && onStop && <button type="button" aria-label="Stop" onClick={onStop}><span aria-hidden="true">■</span>Stop</button>}
      {!streaming && onFeedback && <>
        <button type="button" data-active={feedback === 'up'} aria-label="Helpful" onClick={() => { setFeedback('up'); setShowReasons(false); onFeedback('up') }}><ThumbsUp size={14} /></button>
        <button type="button" data-active={feedback === 'down'} aria-label="Not helpful" onClick={() => { setFeedback('down'); setShowReasons(true); onFeedback('down') }}><ThumbsDown size={14} /></button>
      </>}
      {showReasons && <div className={styles.feedbackReasons} role="group" aria-label="Feedback reason">{feedbackReasons.map(reason => <button key={reason} type="button" onClick={() => { setShowReasons(false); onFeedback?.('down', reason) }}>{reason}</button>)}</div>}
    </div>
    {followUps.length > 0 && !streaming && <div className={styles.followUps}><div className={styles.followUpsLabel}>Continue exploring</div>{followUps.map(item => <button key={item} type="button" className={styles.followUp} onClick={() => onFollowUp?.(item)}>{item}<ChevronRight size={14} /></button>)}</div>}
    <span className={styles.srOnly} role="status">{copyState === 'done' ? 'Answer copied' : ''}</span>
  </section>
}

export function AIAnswerShowcase() {
  const [theme, setTheme] = useState<'light' | 'dark'>('light')
  const [style, setStyle] = useState<'clean' | 'classic'>('clean')
  const [tab, setTab] = useState<'preview' | 'code'>('preview')
  const [answerVersion, setAnswerVersion] = useState(0)
  const [isStreaming, setIsStreaming] = useState(true)
  const [cookieVisible, setCookieVisible] = useState(true)
  const [toast, setToast] = useState('')
  const sources: AnswerSource[] = [
    { name: 'Notion', title: 'Offsite plan – shortlist and budget', snippet: 'Budget €4,500 for three nights; prefer walkable neighbourhoods.' },
    { name: 'Slack', title: '#team-offsite – who can make which dates', snippet: '14–16 May works for everyone, Jonas joins on day two.' },
    { name: 'Wikipedia', title: 'Alfama – the oldest district of Lisbon', href: 'https://en.wikipedia.org/wiki/Alfama', snippet: 'A historic, walkable neighbourhood with quiet mornings.' },
    { name: 'Airbnb', title: 'Two linked houses in Alfama, sleeps 14', snippet: '€2,880 for three nights.' },
    { name: 'Hotels', title: 'Hotels in Baixa for 12 guests', snippet: 'About 20% above the selected houses.' },
    { name: 'Calendar', title: 'Team availability for May', snippet: 'No conflicts on the proposed dates.' },
  ]
  const answer = answerVersion % 2 === 0
    ? '**Alfama** is the best fit for the 12-person offsite: walkable, quiet in the mornings and inside the budget you set [1,3].\n\n- Dates: 14–16 May works for everyone, Jonas joins on day two [2].\n- Stay: two linked houses in Alfama come to €2,880 for three nights, about 20% less than the hotel [4,5].\n- Flights: direct from the team’s main hubs.'
    : '**Alfama** still fits best, with the same dates and a little more room in the budget [1,2].\n\n- Keep the 14–16 May window.\n- Book two linked houses and share the arrival notes.\n- Recheck flight prices before paying.'
  const streamingPreview = '**Alfama** is the best fit for the 12-person offsite: walkable, quiet'
  const notify = (message: string) => { setToast(message); window.setTimeout(() => setToast(''), 1800) }
  return <div className={styles.showcase} data-theme={theme}>
    <div className={styles.showcaseLayout}>
      <aside className={styles.sidebar}>
        <div className={styles.brand}><span className={styles.brandMark}>a</span><span>affectUI</span><button type="button" className={styles.sidebarCollapse} aria-label="Collapse sidebar"><PanelLeft size={14} /></button></div>
        <div className={styles.sidebarSearch}><Search size={14} /><span>Search...</span><span className={styles.searchShortcut}><Command size={11} />K</span></div>
        <nav aria-label="Component navigation">
          <div className={styles.navTop}><a className={styles.navLink} href="#library"><BookOpen size={14} />Library</a><a className={styles.navLink} href="#docs"><BookOpen size={14} />Docs <ChevronRight size={12} /></a><a className={styles.navLink} href="#design"><Palette size={14} />Design System <ChevronRight size={12} /></a><a className={styles.navLink} href="#pricing"><Tag size={14} />Pricing</a></div>
          <div className={styles.styleRow}><span><Palette size={14} />Style</span><button type="button" onClick={() => setStyle('clean')}>Clean <ChevronDown size={12} /></button></div>
          <div className={styles.componentsHeading}><span><PanelLeft size={14} />Components <small>124</small></span><ChevronDown size={13} /></div>
          <div className={styles.componentList}>{['Agent Checkpoints', 'Agent Crew', 'Agent Cursors', 'Agent Findings', 'Agent Fleet', 'Agent Graph', 'Agent Handoff', 'Agent Inbox'].map(item => <a key={item} className={styles.componentLink} href="#chat">{item}<span className={styles.componentPro}>✦</span></a>)}</div>
        </nav>
      </aside>
      <main className={styles.main}>
        <header className={styles.topbar}>
          <div className={styles.themeToggle} aria-label="Theme"><button type="button" data-active={theme === 'light'} onClick={() => setTheme('light')} aria-label="Light theme"><Sun size={14} /></button><button type="button" data-active={theme === 'dark'} onClick={() => setTheme('dark')} aria-label="Dark theme"><Moon size={14} /></button></div>
          <a className={styles.githubLink} href="https://github.com/flohoeller/affectui"><span aria-hidden="true">●</span> 815</a><a className={styles.proLink} href="#pricing">✦ Get Pro</a><a className={styles.topbarLink} href="#login">Log In</a>
        </header>
        <article className={styles.article}>
          <div className={styles.breadcrumb}><a href="#components">Components</a><span>/</span><a href="#chat">Chat</a><span>/</span><span>AI Answer</span></div>
          <div className={styles.hero}><h1>AI Answer</h1><p>Streamed answer with source cards, citation chips that preview their source, copy, regenerate, feedback and follow-ups.</p></div>
          <a className={styles.installLink} href="#pricing">✦ Get Pro to install</a>
          <div className={styles.previewCard}>
            <div className={styles.previewToolbar}><div className={styles.tabs}><button type="button" data-active={tab === 'preview'} onClick={() => setTab('preview')}>◉ Preview</button><button type="button" data-active={tab === 'code'} onClick={() => setTab('code')}>⌁ Code</button></div><div className={styles.previewActions}><span className={styles.toolbarDot} data-tone="orange" /><span className={styles.toolbarDot} data-tone="blue" /><div className={styles.themeMini}><button type="button" data-active={theme === 'light'} onClick={() => setTheme('light')} aria-label="Light preview"><Sun size={12} /></button><button type="button" data-active={theme === 'dark'} onClick={() => setTheme('dark')} aria-label="Dark preview"><Moon size={12} /></button></div><button type="button" className={styles.iconAction} onClick={() => setAnswerVersion(value => value + 1)} aria-label="Reset preview"><RotateCw size={13} /></button><span className={styles.toolbarDivider} /><button type="button" className={styles.toolPill} onClick={() => notify('Claude prompt opened')}>✦</button><button type="button" className={styles.toolPill} onClick={() => notify('ChatGPT prompt opened')}>◎</button><button type="button" className={styles.toolPill} onClick={() => notify('Cursor prompt opened')}>◈</button><button type="button" className={styles.toolPill} onClick={() => notify('v0 prompt opened')}>⌁</button><button type="button" className={styles.toolPill}>MCP</button><button type="button" className={styles.copyToolbar} onClick={() => notify('Prompt copied')}><Copy size={13} />Copy</button></div></div>
            {tab === 'preview' ? <div className={styles.previewBody} data-style={style}><AIAnswer question="Where should we hold the team offsite in Lisbon?" sources={sources} answer={isStreaming ? streamingPreview : answer} streaming={isStreaming} animate={false} followUps={['What should we book first?', 'Show the budget breakdown']} framed onFollowUp={item => notify(item)} onStop={() => { setIsStreaming(false); notify('Streaming stopped') }} onRegenerate={() => { setIsStreaming(true); setAnswerVersion(value => value + 1); notify('Regenerating answer') }} onFeedback={(value, reason) => notify(reason ? `${value}: ${reason}` : value === 'up' ? 'Thanks for the feedback' : 'Tell us what was off')} onCopy={() => notify('Answer copied')} /></div> : <pre style={{ margin: 0, padding: 24, color: 'inherit', overflow: 'auto', fontSize: 12 }}>{`<AIAnswer\n  question="Where should we hold the team offsite?"\n  sources={sources}\n  answer={answer}\n  streaming={!done}\n  framed\n/>`}</pre>}
          </div>
          <div className={styles.docs}><section><h2>When to use</h2><ul><li>Answers from an assistant that searched your docs, the web or connected tools.</li><li>Research and support products where people need to check the source before acting.</li></ul></section><section><h2>When not to use</h2><ul><li>Casual chat without sources.</li><li>Long documents or code.</li></ul></section><section><h2>API reference</h2><p><code>question</code>, <code>sources</code>, <code>answer</code>, <code>streaming</code>, <code>followUps</code>, <code>onFeedback</code> and <code>onCopy</code> are supported by the local component.</p></section></div>
        </article>
      </main>
    </div>
    {cookieVisible && <aside className={styles.cookieBanner} aria-label="Cookies"><strong>Cookies</strong><p>With your OK we use Google Analytics, Google Ads and the Meta pixel to see how the site is used and to measure our ads. You can change this any time under “Cookie settings” in the footer. <u>Privacy</u></p><div><button type="button" onClick={() => setCookieVisible(false)}>Decline</button><button type="button" onClick={() => setCookieVisible(false)}>Accept</button></div></aside>}
    {toast && <div className={styles.srOnly} role="status">{toast}</div>}
  </div>
}
