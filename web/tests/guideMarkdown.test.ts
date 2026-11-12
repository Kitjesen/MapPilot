import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

import { extractHeadings, renderMarkdown } from '../src/guide/markdown.ts'

test('chat replies render links and inline images while guide images retain their text', () => {
  const markdown = '[查看结果](https://example.com/result)\n\n![地图预览](/maps/preview.png)\n\n![运动回放](https://example.com/motion.gif)'
  const options = { pageId: 'agent', resolveDocumentLink: () => undefined }
  const chat = renderMarkdown(markdown, { ...options, renderImages: true })
  assert.match(chat, /<a href="https:\/\/example.com\/result" target="_blank" rel="noreferrer">查看结果<\/a>/)
  assert.match(chat, /<img src="\/maps\/preview.png" alt="地图预览" loading="lazy"/)
  assert.match(chat, /<img src="https:\/\/example.com\/motion.gif" alt="运动回放"/)
  assert.doesNotMatch(renderMarkdown(markdown, options), /<img/)
})

test('chat media keeps scripts and unsupported image sources as text', () => {
  const html = renderMarkdown('![bad](javascript:alert)\n\n![bad](data:image/svg+xml,test)\n\n<script>alert(1)</script>\n\n[bad](javascript:alert)', {
    pageId: 'agent', resolveDocumentLink: () => undefined, renderImages: true,
  })
  assert.doesNotMatch(html, /<img|<script|href="javascript:/)
  assert.match(html, /&lt;script&gt;/)
  assert.match(html, /<a href="#"/)
})

test('generated API tables preserve union types and mode choices in one cell', () => {
  const markdown = readFileSync(new URL('../../docs/api.md', import.meta.url), 'utf8')
  const html = renderMarkdown(markdown, {
    pageId: 'api',
    resolveDocumentLink: () => undefined,
  })
  const rows = [...html.matchAll(/<tr>[\s\S]*?<\/tr>/g)].map((match) => match[0])

  for (const method of ['set_mode', 'navigate_to', 'navigate_to_deg']) {
    const row = rows.find((candidate) => candidate.includes(`<code>${method}</code>`))
    assert.ok(row, `Missing API row for ${method}`)
    assert.equal((row.match(/<td>/g) ?? []).length, 5)
    if (method === 'set_mode') {
      assert.ok(row.includes('manual | autonomous | estop'))
    } else {
      assert.ok(row.includes('<code>z: float | None</code>'))
    }
  }
})

test('operator runbook preserves its referenced section anchors', () => {
  const markdown = readFileSync(new URL('../../docs/operations.md', import.meta.url), 'utf8')
  const headings = extractHeadings(markdown)
  const html = renderMarkdown(markdown, {
    pageId: 'operations',
    resolveDocumentLink: () => undefined,
  })

  assert.ok(headings.some((heading) => heading.id === 'go2-connection-and-debug-records'))
  assert.match(html, /<h2 id="go2-connection-and-debug-records"[^>]*>Go2 connection and debug records/)
  assert.match(html, /<h2 id="stop-and-recovery"[^>]*>Stop and recovery/)
})
