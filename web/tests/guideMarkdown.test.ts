import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

import { extractHeadings, renderMarkdown } from '../src/guide/markdown.ts'

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
