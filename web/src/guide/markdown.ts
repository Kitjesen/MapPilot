export type Heading = {
  depth: 2 | 3
  id: string
  text: string
}

export type MarkdownRenderOptions = {
  pageId: string
  resolveDocumentLink: (href: string) => string | undefined
}

type RawHeading = {
  depth: number
  id: string
  text: string
}

const HTML_ESCAPE = /[&<>"']/g

const ESCAPED_VALUES: Record<string, string> = {
  '&': '&amp;',
  '<': '&lt;',
  '>': '&gt;',
  '"': '&quot;',
  "'": '&#039;',
}

function escapeHtml(value: string): string {
  return value.replace(HTML_ESCAPE, (character) => ESCAPED_VALUES[character])
}

export function slugify(value: string): string {
  return value
    .trim()
    .toLowerCase()
    .replace(/[`*_~]/g, '')
    .replace(/[^\w\u4e00-\u9fff-]+/g, '-')
    .replace(/^-+|-+$/g, '') || 'section'
}

function collectRawHeadings(markdown: string): RawHeading[] {
  const occurrences = new Map<string, number>()
  const headings: RawHeading[] = []

  for (const line of markdown.replace(/\r\n/g, '\n').split('\n')) {
    const match = /^(#{1,3})\s+(.+?)\s*$/.exec(line)
    if (!match) {
      continue
    }

    const text = match[2].replace(/\s+#*$/, '')
    const base = slugify(text)
    const nextOccurrence = (occurrences.get(base) ?? 0) + 1
    occurrences.set(base, nextOccurrence)
    headings.push({
      depth: match[1].length,
      id: nextOccurrence === 1 ? base : `${base}-${nextOccurrence}`,
      text,
    })
  }

  return headings
}

export function extractHeadings(markdown: string): Heading[] {
  return collectRawHeadings(markdown)
    .filter((heading): heading is RawHeading & { depth: 2 | 3 } => heading.depth === 2 || heading.depth === 3)
    .map(({ depth, id, text }) => ({ depth, id, text }))
}

function isTableDivider(line: string): boolean {
  return /^\s*\|?\s*:?-{3,}:?\s*(\|\s*:?-{3,}:?\s*)+\|?\s*$/.test(line)
}

function splitTableRow(line: string): string[] {
  return line
    .trim()
    .replace(/^\|/, '')
    .replace(/\|$/, '')
    .split('|')
    .map((cell) => cell.trim())
}

function isListItem(line: string): boolean {
  return /^\s*(?:[-+*]|\d+\.)\s+/.test(line)
}

function inlineMarkdown(value: string, options: MarkdownRenderOptions): string {
  let html = escapeHtml(value)

  html = html.replace(/&lt;br\s*\/?&gt;/gi, '<br>')
  html = html.replace(/`([^`]+)`/g, '<code>$1</code>')
  html = html.replace(/!\[([^\]]*)\]\(([^)\s]+)\)/g, '<span class="markdown-image-alt">$1</span>')
  html = html.replace(/\[([^\]]+)\]\(([^)]+)\)/g, (_match, label: string, target: string) => {
    const guideTarget = options.resolveDocumentLink(target)
    if (!guideTarget && isLocalSourceReference(target)) {
      return `<span class="source-reference" title="Repository source: ${escapeHtml(target)}">${label}<span aria-hidden="true">source</span></span>`
    }
    const href = guideTarget ?? safeHref(target)
    const external = /^https?:\/\//.test(href)
    const attributes = external ? ' target="_blank" rel="noreferrer"' : ''
    return `<a href="${href}"${attributes}>${label}</a>`
  })
  html = html.replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>')
  html = html.replace(/(^|[^*])\*([^*]+)\*/g, '$1<em>$2</em>')
  return html
}

function safeHref(href: string): string {
  const trimmed = href.trim()
  if (/^(?:https?:|mailto:|#|\.\/|\.\.\/|\/)/.test(trimmed)) {
    return trimmed.replace(/"/g, '%22')
  }
  return '#'
}

function isLocalSourceReference(href: string): boolean {
  const trimmed = href.trim()
  return /^(?:\.\/|\.\.\/|\/)/.test(trimmed) && !trimmed.startsWith('#')
}

function calloutClass(text: string): string {
  const normalized = text.toLowerCase()
  if (/(danger|危险|禁止|motion-capable|可能驱动)/.test(normalized)) {
    return 'is-danger'
  }
  if (/(caution|warning|注意|警告|停止条件)/.test(normalized)) {
    return 'is-caution'
  }
  return 'is-note'
}

export function renderMarkdown(markdown: string, options: MarkdownRenderOptions): string {
  const lines = markdown.replace(/\r\n/g, '\n').split('\n')
  const headings = collectRawHeadings(markdown)
  let headingIndex = 0
  let codeIndex = 0
  let html = ''
  let paragraph: string[] = []
  let listItems: string[] = []
  let listType: 'ol' | 'ul' | undefined

  const flushParagraph = () => {
    if (paragraph.length > 0) {
      html += `<p>${inlineMarkdown(paragraph.join(' '), options)}</p>`
      paragraph = []
    }
  }

  const flushList = () => {
    if (listItems.length > 0 && listType) {
      html += `<${listType}>${listItems.map((item) => `<li>${inlineMarkdown(item, options)}</li>`).join('')}</${listType}>`
      listItems = []
      listType = undefined
    }
  }

  for (let index = 0; index < lines.length; index += 1) {
    const line = lines[index]
    const codeFence = /^```([^\s]*)\s*$/.exec(line)
    if (codeFence) {
      flushParagraph()
      flushList()
      const codeLines: string[] = []
      index += 1
      while (index < lines.length && !/^```\s*$/.test(lines[index])) {
        codeLines.push(lines[index])
        index += 1
      }
      codeIndex += 1
      const language = codeFence[1] || 'text'
      html += `<div class="code-block"><div class="code-toolbar"><span>${escapeHtml(language)}</span><button type="button" data-copy="code-${codeIndex}" aria-label="复制代码">复制</button></div><pre><code>${escapeHtml(codeLines.join('\n'))}</code></pre></div>`
      continue
    }

    if (/^(?: {4}|\t)/.test(line)) {
      flushParagraph()
      flushList()
      const codeLines: string[] = []
      while (index < lines.length && /^(?: {4}|\t)/.test(lines[index])) {
        codeLines.push(lines[index].replace(/^(?: {4}|\t)/, ''))
        index += 1
      }
      index -= 1
      codeIndex += 1
      html += `<div class="code-block"><div class="code-toolbar"><span>shell</span><button type="button" data-copy="code-${codeIndex}" aria-label="复制代码">复制</button></div><pre><code>${escapeHtml(codeLines.join('\n'))}</code></pre></div>`
      continue
    }

    const headingMatch = /^(#{1,3})\s+(.+?)\s*$/.exec(line)
    if (headingMatch) {
      flushParagraph()
      flushList()
      const heading = headings[headingIndex]
      headingIndex += 1
      const depth = Math.min(headingMatch[1].length, 3)
      const headingText = headingMatch[2].replace(/\s+#*$/, '')
      const link = `#${options.pageId}::${heading.id}`
      html += `<h${depth} id="${heading.id}" tabindex="-1">${inlineMarkdown(headingText, options)}<a class="heading-anchor" href="${link}" aria-label="复制 ${escapeHtml(headingText)} 的链接">#</a></h${depth}>`
      continue
    }

    if (line.trim() === '') {
      flushParagraph()
      flushList()
      continue
    }

    if (/^---+$/.test(line.trim())) {
      flushParagraph()
      flushList()
      html += '<hr />'
      continue
    }

    if (line.startsWith('>')) {
      flushParagraph()
      flushList()
      const quoteLines: string[] = []
      while (index < lines.length && lines[index].startsWith('>')) {
        quoteLines.push(lines[index].replace(/^>\s?/, ''))
        index += 1
      }
      index -= 1
      const quote = quoteLines.join(' ')
      html += `<aside class="callout ${calloutClass(quote)}"><p>${inlineMarkdown(quote, options)}</p></aside>`
      continue
    }

    if (index + 1 < lines.length && line.includes('|') && isTableDivider(lines[index + 1])) {
      flushParagraph()
      flushList()
      const headers = splitTableRow(line)
      const rows: string[][] = []
      index += 2
      while (index < lines.length && lines[index].includes('|') && lines[index].trim() !== '') {
        rows.push(splitTableRow(lines[index]))
        index += 1
      }
      index -= 1
      html += '<div class="table-scroll"><table><thead><tr>'
      html += headers.map((header) => `<th>${inlineMarkdown(header, options)}</th>`).join('')
      html += '</tr></thead><tbody>'
      html += rows.map((row) => `<tr>${headers.map((_, cellIndex) => `<td>${inlineMarkdown(row[cellIndex] ?? '', options)}</td>`).join('')}</tr>`).join('')
      html += '</tbody></table></div>'
      continue
    }

    if (isListItem(line)) {
      flushParagraph()
      const match = /^\s*(?:([-+*])|(\d+)\.)\s+(.+)$/.exec(line)
      if (!match) {
        continue
      }
      const nextType: 'ol' | 'ul' = match[2] ? 'ol' : 'ul'
      if (listType && listType !== nextType) {
        flushList()
      }
      listType = nextType
      listItems.push(match[3])
      continue
    }

    paragraph.push(line.trim())
  }

  flushParagraph()
  flushList()
  return html
}
