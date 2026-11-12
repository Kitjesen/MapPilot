import assert from 'node:assert/strict'
import { createRequire } from 'node:module'
import test from 'node:test'
import { conversationDependency } from './agentConversationFixtures.ts'

const require = createRequire(import.meta.url)
const { createElement } = require('react')
const { renderToStaticMarkup } = require('react-dom/server')
const { ToolActivity, ToolActivityGroup } = conversationDependency('/agentConversation/ToolActivity.tsx') as Record<string, unknown>
const message = (kind: string, result: unknown) => ({ id: kind, seq: 1, turn_id: 't', role: 'tool', kind,
  text: JSON.stringify(result), metadata: { status: 'completed', duration_ms: 1480 } })

test('web search shows source links and own timing without opening raw JSON', () => {
  const html = renderToStaticMarkup(createElement(ToolActivity, { message: message('web_search', { sources: [
    { title: 'Official <robot>', url: 'https://example.com', content: 'Search snippet' },
    { title: 'unsafe', url: 'javascript:alert(1)', content: '' },
  ] }) }))
  assert.match(html, /搜索来源/)
  assert.match(html, /Official &lt;robot&gt;/)
  assert.match(html, /href="https:\/\/example.com"/)
  assert.match(html, /1.5 秒/)
  assert.doesNotMatch(html, /javascript:|unsafe/)
})

test('extract is expandable and failures do not render as sources', () => {
  const html = renderToStaticMarkup(createElement(ToolActivity, { message: message('web_extract', { sources: [
    { url: 'https://example.com', content: 'Page body', truncated: true },
  ] }) }))
  assert.match(html, /展开正文/)
  assert.match(html, /内容已截断/)
  const failed = renderToStaticMarkup(createElement(ToolActivity, { message: message('web_search', { error: 'TAVILY_API_KEY missing' }) }))
  assert.match(failed, /未完成/)
  assert.doesNotMatch(failed, /搜索来源/)
})

test('sources stay visible after a long tool chain', () => {
  const messages = [0, 1, 2, 3].map(i => ({ ...message('web_search', { sources: [] }), id: String(i) }))
  const html = renderToStaticMarkup(createElement(ToolActivityGroup, { messages, active: false, awaiting: [] }))
  assert.doesNotMatch(html, /展开工具过程/)
})
