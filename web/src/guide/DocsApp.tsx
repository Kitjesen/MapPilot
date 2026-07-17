import { useEffect, useMemo, useRef, useState, type MouseEvent as ReactMouseEvent } from 'react'
import {
  ArrowLeft,
  ArrowRight,
  Bot,
  ChevronDown,
  ChevronRight,
  CircleHelp,
  Code2,
  ExternalLink,
  FileText,
  HeartPulse,
  Map,
  Menu,
  Rocket,
  Route,
  Search,
  Server,
  ShieldCheck,
  TriangleAlert,
  Wrench,
  X,
} from 'lucide-react'

import {
  findDocument,
  findDocumentBySourcePath,
  guideDocuments,
  guideGroups,
  type GuideDocument,
  type OperationBoundary,
} from './docsRegistry'
import { extractHeadings, renderMarkdown, slugify, type Heading } from './markdown'

type GuideLocation = {
  documentId: string
  headingId?: string
}

type SearchResult = {
  document: GuideDocument
  section?: string
  excerpt: string
}

const DEFAULT_LOCATION: GuideLocation = { documentId: 'home' }

function parseLocation(): GuideLocation {
  const fragment = decodeURIComponent(window.location.hash.slice(1))
  if (!fragment) {
    return DEFAULT_LOCATION
  }
  const [documentId, headingId] = fragment.split('::')
  if (!guideDocuments.some((document) => document.id === documentId)) {
    return DEFAULT_LOCATION
  }
  return { documentId, headingId }
}

function toHash(location: GuideLocation): string {
  return `#${location.documentId}${location.headingId ? `::${location.headingId}` : ''}`
}

function readableText(markdown: string): string {
  return markdown
    .replace(/```[\s\S]*?```/g, ' ')
    .replace(/!?(?:\[([^\]]*)\])\([^)]+\)/g, '$1')
    .replace(/[`*_>#|]/g, ' ')
    .replace(/\s+/g, ' ')
    .trim()
}

function findSearchResults(query: string): SearchResult[] {
  const normalizedQuery = query.trim().toLocaleLowerCase()
  if (!normalizedQuery) {
    return []
  }

  return guideDocuments
    .map((document): SearchResult | undefined => {
      const content = readableText(document.content)
      const searchable = `${document.title} ${document.description} ${content}`.toLocaleLowerCase()
      const matchIndex = searchable.indexOf(normalizedQuery)
      if (matchIndex === -1) {
        return undefined
      }

      const lowerContent = content.toLocaleLowerCase()
      const contentIndex = lowerContent.indexOf(normalizedQuery)
      const excerptStart = Math.max(0, contentIndex - 54)
      const excerpt = content.slice(excerptStart, excerptStart + 148)
      const section = extractHeadings(document.content).find((heading) =>
        heading.text.toLocaleLowerCase().includes(normalizedQuery),
      )?.text
      return {
        document,
        ...(section ? { section } : {}),
        excerpt: `${excerpt}${excerpt.length < content.length ? '…' : ''}`,
      }
    })
    .filter((result): result is SearchResult => result !== undefined)
    .slice(0, 12)
}

function operationIcon(operation: OperationBoundary) {
  if (operation === 'Motion-capable') {
    return <TriangleAlert aria-hidden="true" />
  }
  if (operation === 'State-changing') {
    return <Wrench aria-hidden="true" />
  }
  return <ShieldCheck aria-hidden="true" />
}

function operationLabel(operation: OperationBoundary): string {
  if (operation === 'Motion-capable') {
    return '可能产生运动'
  }
  if (operation === 'State-changing') {
    return '会改变系统状态'
  }
  return '仅阅读与检查'
}

function iconForDocument(documentId: string) {
  const commonProps = { 'aria-hidden': true }
  if (documentId === 'task-guides' || documentId === 'operations') {
    return <Route {...commonProps} />
  }
  if (documentId === 'safety') {
    return <ShieldCheck {...commonProps} />
  }
  if (documentId === 'integrations') {
    return <Server {...commonProps} />
  }
  if (documentId === 'development' || documentId === 'architecture') {
    return <Code2 {...commonProps} />
  }
  if (documentId === 'deployment') {
    return <Rocket {...commonProps} />
  }
  if (documentId === 'quick-start' || documentId === 'getting-started') {
    return <Bot {...commonProps} />
  }
  return <FileText {...commonProps} />
}

function normalizeLinkPath(fromSourcePath: string, href: string): { sourcePath: string; headingId?: string } | undefined {
  if (!href.includes('.md')) {
    return undefined
  }
  const [filePart, fragment] = href.split('#')
  try {
    const base = new URL(`https://lingtu.invalid/${fromSourcePath}`)
    const resolved = new URL(filePart, base)
    const sourcePath = resolved.pathname.replace(/^\//, '')
    return { sourcePath, headingId: fragment ? slugify(fragment) : undefined }
  } catch {
    return undefined
  }
}

function resolveDocumentLink(fromDocument: GuideDocument, href: string): string | undefined {
  if (href.startsWith('#')) {
    return `#${fromDocument.id}::${slugify(href.slice(1))}`
  }
  const normalized = normalizeLinkPath(fromDocument.sourcePath, href)
  if (!normalized) {
    return undefined
  }
  const target = findDocumentBySourcePath(normalized.sourcePath)
  return target ? toHash({ documentId: target.id, headingId: normalized.headingId }) : undefined
}

function navigateTo(location: GuideLocation): void {
  window.location.hash = toHash(location)
}

function Sidebar({
  activeDocumentId,
  isMobileOpen,
  onClose,
}: {
  activeDocumentId: string
  isMobileOpen: boolean
  onClose: () => void
}) {
  const [openGroups, setOpenGroups] = useState<Set<string>>(() => new Set(guideGroups.map((group) => group.label)))
  const visibleOpenGroups = useMemo(() => {
    const next = new Set(openGroups)
    const activeGroup = guideGroups.find((group) => group.ids.includes(activeDocumentId))
    if (activeGroup) {
      next.add(activeGroup.label)
    }
    return next
  }, [activeDocumentId, openGroups])

  return (
    <aside className={`docs-sidebar ${isMobileOpen ? 'is-open' : ''}`} aria-label="文档导航">
      <div className="sidebar-mobile-heading">
        <span>文档导航</span>
        <button type="button" className="icon-button" onClick={onClose} aria-label="关闭文档导航">
          <X aria-hidden="true" />
        </button>
      </div>
      <nav>
        <a
          href="#home"
          className={`sidebar-home ${activeDocumentId === 'home' ? 'is-active' : ''}`}
          aria-current={activeDocumentId === 'home' ? 'page' : undefined}
          onClick={onClose}
        >
          <span className="sidebar-icon"><Map aria-hidden="true" /></span>
          文档概览
        </a>
        {guideGroups.map((group) => {
          const isOpen = visibleOpenGroups.has(group.label)
          return (
            <section className="sidebar-group" key={group.label}>
              <button
                type="button"
                className="sidebar-group-toggle"
                aria-expanded={isOpen}
                onClick={() => {
                  setOpenGroups((current) => {
                    const next = new Set(current)
                    if (next.has(group.label)) {
                      next.delete(group.label)
                    } else {
                      next.add(group.label)
                    }
                    return next
                  })
                }}
              >
                <span>{group.label}</span>
                <ChevronDown aria-hidden="true" className={isOpen ? 'is-open' : ''} />
              </button>
              {isOpen && (
                <div className="sidebar-links">
                  {group.ids.map((documentId) => {
                    const document = findDocument(documentId)
                    const active = documentId === activeDocumentId
                    return (
                      <a
                        href={toHash({ documentId })}
                        key={documentId}
                        className={active ? 'is-active' : ''}
                        aria-current={active ? 'page' : undefined}
                        onClick={onClose}
                      >
                        {document.title}
                      </a>
                    )
                  })}
                </div>
              )}
            </section>
          )
        })}
      </nav>
      <div className="sidebar-footer">
        <a href="#current" onClick={onClose}><CircleHelp aria-hidden="true" /> 文档状态说明</a>
        <a href="#known-gaps" onClick={onClose}><HeartPulse aria-hidden="true" /> 已知限制</a>
      </div>
    </aside>
  )
}

function Header({
  onMenuOpen,
  onSearchOpen,
}: {
  onMenuOpen: () => void
  onSearchOpen: () => void
}) {
  return (
    <header className="docs-header">
      <button type="button" className="icon-button mobile-menu-button" onClick={onMenuOpen} aria-label="打开文档导航">
        <Menu aria-hidden="true" />
      </button>
      <a className="brand" href="#home" aria-label="LingTu Docs 首页">
        <span className="brand-mark" aria-hidden="true"><span /></span>
        <span>LingTu</span>
        <span className="brand-divider" aria-hidden="true" />
        <span className="brand-docs">Docs</span>
      </a>
      <button type="button" className="search-trigger" onClick={onSearchOpen} aria-label="搜索文档，快捷键 Control 或 Command 加 K">
        <Search aria-hidden="true" />
        <span>搜索文档、命令和概念</span>
        <kbd>Ctrl / ⌘ K</kbd>
      </button>
      <div className="header-actions">
        <a href="#current">文档状态</a>
        <a href="#reference" aria-label="打开文档参考">参考 <ExternalLink aria-hidden="true" /></a>
      </div>
    </header>
  )
}

function Metadata({ document }: { document: GuideDocument }) {
  return (
    <dl className="doc-metadata" aria-label="页面信息">
      <div>
        <dt>状态</dt>
        <dd>{document.status}</dd>
      </div>
      <div>
        <dt>适用</dt>
        <dd>{document.audience}</dd>
      </div>
      <div>
        <dt>环境</dt>
        <dd>{document.runsOn}</dd>
      </div>
      <div className={`operation-${document.operation.toLowerCase().replace(/[^a-z]+/g, '-')}`}>
        <dt>执行边界</dt>
        <dd>{operationIcon(document.operation)}<span>{operationLabel(document.operation)}</span></dd>
      </div>
    </dl>
  )
}

function HomePage({ onNavigate }: { onNavigate: (documentId: string) => void }) {
  const paths = [
    { id: 'quick-start', title: '本地或仿真首跑', text: '先证明模块图和选定的仿真边界，而不是直接连接现场机器人。', icon: <Rocket aria-hidden="true" /> },
    { id: 'task-guides', title: '按任务执行', text: '建立地图、检查路径、导航、语义任务与探索各自有清晰的前置条件。', icon: <Route aria-hidden="true" /> },
    { id: 'operations', title: '部署与运维', text: '通过健康检查、路线预览和恢复路径管理实际运行的系统。', icon: <Wrench aria-hidden="true" /> },
  ]

  const capabilities = [
    { title: '理解环境', text: '本地、仿真和现场证明不同的事情。', icon: <Map aria-hidden="true" /> },
    { title: '保持边界', text: '模块、蓝图、端口和显式连线定义产品路径。', icon: <Code2 aria-hidden="true" /> },
    { title: '安全行动', text: '运动前先检查定位、地图、路线和命令源。', icon: <ShieldCheck aria-hidden="true" /> },
  ]

  return (
    <div className="home-page">
      <section className="home-hero" aria-labelledby="home-title">
        <div className="hero-copy">
          <p className="eyebrow">LingTu documentation</p>
          <h1 id="home-title">让四足机器人可靠地理解、建图与导航。</h1>
          <p className="hero-lede">从可复现的本地验证开始，逐步进入仿真、部署和受监督的现场任务。每一步都说明它真正证明什么，以及何时必须停止。</p>
          <div className="hero-actions">
            <a className="button-primary" href="#quick-start">开始使用 <ArrowRight aria-hidden="true" /></a>
            <a className="text-link" href="#concepts">理解架构 <ArrowRight aria-hidden="true" /></a>
          </div>
        </div>
        <div className="hero-diagram" aria-label="从传感器到安全命令边界的 LingTu 数据流示意">
          <div className="diagram-grid" aria-hidden="true" />
          <div className="diagram-node node-sensors">感知</div>
          <div className="diagram-node node-map">地图</div>
          <div className="diagram-node node-plan">规划</div>
          <div className="diagram-node node-safety">安全</div>
          <div className="diagram-route" aria-hidden="true"><span /><span /><span /></div>
          <div className="diagram-robot" aria-hidden="true"><span /><span /><span /><span /></div>
          <p>传感器 → 定位 → 地图 → 规划 → 安全 → 驱动</p>
        </div>
      </section>

      <section className="home-routes" aria-labelledby="paths-title">
        <div className="section-heading">
          <p className="eyebrow">Choose a path</p>
          <h2 id="paths-title">按你要完成的事开始</h2>
        </div>
        <div className="route-grid">
          {paths.map((path, index) => (
            <a className="route-card" href={`#${path.id}`} key={path.id}>
              <span className="route-number">0{index + 1}</span>
              <span className="route-icon">{path.icon}</span>
              <h3>{path.title}</h3>
              <p>{path.text}</p>
              <span className="route-arrow" aria-hidden="true"><ArrowRight /></span>
            </a>
          ))}
        </div>
      </section>

      <section className="home-capabilities" aria-labelledby="capabilities-title">
        <div className="section-heading">
          <p className="eyebrow">Product model</p>
          <h2 id="capabilities-title">把系统理解成可验证的边界</h2>
        </div>
        <div className="capability-list">
          {capabilities.map((capability) => (
            <article key={capability.title}>
              <span>{capability.icon}</span>
              <div><h3>{capability.title}</h3><p>{capability.text}</p></div>
            </article>
          ))}
        </div>
      </section>

      <section className="home-safety" aria-label="现场操作提示">
        <TriangleAlert aria-hidden="true" />
        <div>
          <strong>现场操作不是 Quick Start 的默认下一步。</strong>
          <p>任何可能产生运动的流程都要先完成无运动检查、地图与定位验证、路线预览和操作员监督。</p>
        </div>
        <button type="button" onClick={() => onNavigate('safety')}>查看安全边界 <ChevronRight aria-hidden="true" /></button>
      </section>
    </div>
  )
}

function OnThisPage({
  document,
  headings,
  activeHeadingId,
}: {
  document: GuideDocument
  headings: Heading[]
  activeHeadingId?: string
}) {
  return (
    <aside className="on-this-page" aria-label="本页目录">
      <section>
        <p className="side-label">本页目录</p>
        <nav>
          {headings.length > 0 ? headings.map((heading) => (
            <a
              href={toHash({ documentId: document.id, headingId: heading.id })}
              className={`${heading.depth === 3 ? 'is-nested' : ''} ${activeHeadingId === heading.id ? 'is-active' : ''}`}
              key={heading.id}
            >
              {heading.text}
            </a>
          )) : <span className="empty-outline">此页没有分节目录。</span>}
        </nav>
      </section>
      <section className="page-facts">
        <p className="side-label">页面信息</p>
        <dl>
          <div><dt>来源</dt><dd><code>{document.contentSourcePath ?? document.sourcePath}</code></dd></div>
          <div><dt>状态</dt><dd>{document.status}</dd></div>
          <div><dt>文档索引</dt><dd>{document.lastVerified}</dd></div>
        </dl>
      </section>
      <section className={`boundary-note operation-${document.operation.toLowerCase().replace(/[^a-z]+/g, '-')}`}>
        {operationIcon(document.operation)}
        <div><strong>{document.operation}</strong><span>{operationLabel(document.operation)}</span></div>
      </section>
    </aside>
  )
}

function SearchDialog({
  isOpen,
  onClose,
  onNavigate,
}: {
  isOpen: boolean
  onClose: () => void
  onNavigate: (documentId: string) => void
}) {
  const [query, setQuery] = useState('')
  const [selectedIndex, setSelectedIndex] = useState(0)
  const inputRef = useRef<HTMLInputElement>(null)
  const results = useMemo(() => findSearchResults(query), [query])

  useEffect(() => {
    if (isOpen) {
      window.setTimeout(() => inputRef.current?.focus(), 0)
    }
  }, [isOpen])

  if (!isOpen) {
    return null
  }

  const choose = (result: SearchResult | undefined) => {
    if (!result) {
      return
    }
    onNavigate(result.document.id)
    onClose()
  }

  return (
    <div className="search-backdrop" role="presentation" onMouseDown={onClose}>
      <section
        className="search-dialog"
        role="dialog"
        aria-modal="true"
        aria-label="搜索文档"
        onMouseDown={(event) => event.stopPropagation()}
      >
        <div className="search-dialog-input">
          <Search aria-hidden="true" />
          <input
            ref={inputRef}
            value={query}
            onChange={(event) => {
              setQuery(event.target.value)
              setSelectedIndex(0)
            }}
            onKeyDown={(event) => {
              if (event.key === 'ArrowDown') {
                event.preventDefault()
                setSelectedIndex((index) => Math.min(index + 1, Math.max(0, results.length - 1)))
              }
              if (event.key === 'ArrowUp') {
                event.preventDefault()
                setSelectedIndex((index) => Math.max(0, index - 1))
              }
              if (event.key === 'Enter') {
                event.preventDefault()
                choose(results[selectedIndex])
              }
            }}
            placeholder="搜索文档、命令和概念"
            aria-label="搜索文档、命令和概念"
          />
          <kbd>Esc</kbd>
        </div>
        <div className="search-results" role="listbox" aria-label="搜索结果">
          {query && results.length === 0 && <p className="empty-search">没有找到匹配内容。尝试更短的关键词、命令或页面标题。</p>}
          {!query && <p className="empty-search">搜索页面标题、章节、命令和正文。使用 ↑ ↓ 选择，Enter 打开。</p>}
          {results.map((result, index) => (
            <button
              type="button"
              role="option"
              aria-selected={index === selectedIndex}
              className={index === selectedIndex ? 'is-selected' : ''}
              key={result.document.id}
              onMouseEnter={() => setSelectedIndex(index)}
              onClick={() => choose(result)}
            >
              <span className="search-result-icon">{iconForDocument(result.document.id)}</span>
              <span><strong>{result.document.title}</strong>{result.section && <em>{result.section}</em>}<small>{result.excerpt}</small></span>
              <ChevronRight aria-hidden="true" />
            </button>
          ))}
        </div>
        <footer><span><kbd>↑</kbd><kbd>↓</kbd> 选择</span><span><kbd>↵</kbd> 打开</span><span><kbd>Esc</kbd> 关闭</span></footer>
      </section>
    </div>
  )
}

function stripLeadingH1(markdown: string): string {
  return markdown
    .replace(/^#\s+.+?\n+/m, '')
    .replace(/^> \*\*Status:[^\n]*(?:\n> [^\n]*)*\n\n/, '')
}

export function DocsApp() {
  const [location, setLocation] = useState<GuideLocation>(parseLocation)
  const [isSearchOpen, setIsSearchOpen] = useState(false)
  const [isMobileNavOpen, setIsMobileNavOpen] = useState(false)
  const currentDocument = findDocument(location.documentId)
  const headings = useMemo(() => extractHeadings(currentDocument.content), [currentDocument.content])
  const renderedContent = useMemo(
    () => renderMarkdown(stripLeadingH1(currentDocument.content), {
      pageId: currentDocument.id,
      resolveDocumentLink: (href) => resolveDocumentLink(currentDocument, href),
    }),
    [currentDocument],
  )
  const visibleDocuments = guideDocuments.filter((entry) => entry.id !== 'home' && !entry.hidden)
  const documentIndex = visibleDocuments.findIndex((entry) => entry.id === currentDocument.id)
  const previousDocument = documentIndex > 0 ? visibleDocuments[documentIndex - 1] : undefined
  const nextDocument = documentIndex >= 0 && documentIndex < visibleDocuments.length - 1 ? visibleDocuments[documentIndex + 1] : undefined

  useEffect(() => {
    const updateLocation = () => setLocation(parseLocation())
    window.addEventListener('hashchange', updateLocation)
    return () => window.removeEventListener('hashchange', updateLocation)
  }, [])

  useEffect(() => {
    if (!location.headingId) {
      window.scrollTo({ top: 0, behavior: 'auto' })
      return
    }
    window.setTimeout(() => {
      window.document.getElementById(location.headingId ?? '')?.scrollIntoView({ behavior: 'smooth', block: 'start' })
    }, 0)
  }, [currentDocument.id, location.headingId])

  useEffect(() => {
    const onKeyDown = (event: KeyboardEvent) => {
      const target = event.target as HTMLElement | null
      const targetIsEditable = target?.matches('input, textarea, [contenteditable="true"]')
      if ((event.metaKey || event.ctrlKey) && event.key.toLowerCase() === 'k') {
        event.preventDefault()
        setIsSearchOpen(true)
      }
      if (event.key === '/' && !targetIsEditable) {
        event.preventDefault()
        setIsSearchOpen(true)
      }
      if (event.key === 'Escape') {
        setIsSearchOpen(false)
        setIsMobileNavOpen(false)
      }
    }
    window.addEventListener('keydown', onKeyDown)
    return () => window.removeEventListener('keydown', onKeyDown)
  }, [])

  const handleContentClick = (event: ReactMouseEvent<HTMLElement>) => {
    const target = event.target as HTMLElement
    const copyButton = target.closest<HTMLButtonElement>('button[data-copy]')
    if (copyButton) {
      const code = copyButton.parentElement?.parentElement?.querySelector('code')?.textContent
      if (code) {
        void navigator.clipboard.writeText(code).then(() => {
          const originalText = copyButton.textContent
          copyButton.textContent = '已复制'
          copyButton.dataset.copied = 'true'
          window.setTimeout(() => {
            copyButton.textContent = originalText
            delete copyButton.dataset.copied
          }, 1600)
        })
      }
      return
    }
    const link = target.closest<HTMLAnchorElement>('a[href^="#"]')
    if (link) {
      event.preventDefault()
      window.location.hash = link.getAttribute('href') ?? '#home'
    }
  }

  const navigate = (documentId: string) => {
    navigateTo({ documentId })
  }

  return (
    <div className="docs-app">
      <a className="skip-link" href="#main-content">跳至正文</a>
      <Header onMenuOpen={() => setIsMobileNavOpen(true)} onSearchOpen={() => setIsSearchOpen(true)} />
      <div className="docs-layout">
        <Sidebar activeDocumentId={currentDocument.id} isMobileOpen={isMobileNavOpen} onClose={() => setIsMobileNavOpen(false)} />
        {isMobileNavOpen && <button type="button" className="sidebar-scrim" aria-label="关闭文档导航" onClick={() => setIsMobileNavOpen(false)} />}
        <main id="main-content" className="docs-main">
          {currentDocument.id === 'home' ? (
            <HomePage onNavigate={navigate} />
          ) : (
            <>
              <header className="document-header">
                <nav className="breadcrumbs" aria-label="面包屑">
                  <a href="#home">文档</a><ChevronRight aria-hidden="true" /><span>{currentDocument.group}</span><ChevronRight aria-hidden="true" /><strong>{currentDocument.title}</strong>
                </nav>
                <p className="eyebrow">{currentDocument.group}</p>
                <h1>{currentDocument.title}</h1>
                <p className="document-lede">{currentDocument.description}</p>
                <Metadata document={currentDocument} />
              </header>
              <article className="markdown-content" onClick={handleContentClick} dangerouslySetInnerHTML={{ __html: renderedContent }} />
              <nav className="document-pagination" aria-label="相邻文档">
                {previousDocument ? <a href={toHash({ documentId: previousDocument.id })}><ArrowLeft aria-hidden="true" /><span><small>上一篇</small>{previousDocument.title}</span></a> : <span />}
                {nextDocument ? <a href={toHash({ documentId: nextDocument.id })}><span><small>下一篇</small>{nextDocument.title}</span><ArrowRight aria-hidden="true" /></a> : <span />}
              </nav>
            </>
          )}
        </main>
        {currentDocument.id !== 'home' && <OnThisPage document={currentDocument} headings={headings} activeHeadingId={location.headingId} />}
      </div>
      {isSearchOpen && <SearchDialog isOpen={isSearchOpen} onClose={() => setIsSearchOpen(false)} onNavigate={navigate} />}
    </div>
  )
}
