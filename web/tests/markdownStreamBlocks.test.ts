import assert from 'node:assert/strict'
import test from 'node:test'
import { renderMarkdown } from '../src/guide/markdown.ts'
import { replyMarkdownBlocks } from '../src/components/agentConversation/markdownStreamBlocks.ts'

const render = (text: string) => renderMarkdown(text, { pageId: 'agent', resolveDocumentLink: () => undefined, renderImages: true })

test('streamed prose keeps completed blocks stable and preserves Markdown semantics', () => {
  const complete = '已读取当前地图。\n\n接下来检查通行区域。\n\n最后核对结果。'
  const first = replyMarkdownBlocks(complete.slice(0, complete.indexOf('最后')))
  const final = replyMarkdownBlocks(complete)
  assert.equal(first[0], final[0])
  assert.equal(first[1], final[1])
  for (let end = 1; end <= complete.length; end++) {
    const prefix = complete.slice(0, end)
    assert.equal(replyMarkdownBlocks(prefix).join(''), prefix)
    assert.equal(replyMarkdownBlocks(prefix).map(render).join(''), render(prefix))
  }
  const rewritten = '当前地图暂时不可读。\n\n请检查连接。'
  assert.equal(replyMarkdownBlocks(rewritten).map(render).join(''), render(rewritten))
})

test('lists, quotes, tables and unsafe source retain the existing renderer output', () => {
  for (const source of [
    '- 一\n- 二\n\n  补充内容\n\n接下来',
    '> 引用\n> 两行\n\n普通正文',
    '| 项目 | 结果 |\n| --- | --- |\n| 地图 | 待检查 |\n\n下一段',
    '<img src=x onerror=alert(1)>\n\n[链接](javascript:alert(1))',
    '正文\n\n[id]: https://example.com\n\n[id]',
  ]) for (let end = 1; end <= source.length; end++) {
    const prefix = source.slice(0, end)
    assert.equal(replyMarkdownBlocks(prefix).map(render).join(''), render(prefix), prefix)
  }
})

test('document numbering and fence syntax use the unchanged full renderer', () => {
  for (const source of [
    '## 同名\n\n一\n\n## 同名\n\n二',
    '正文\n\n```js\na()\n```\n\n```sh\nb\n```',
    '````\na\n````\n\nb',
    '~~~\na\n~~~\n\nb',
  ]) {
    assert.deepEqual(replyMarkdownBlocks(source), [source])
    assert.equal(replyMarkdownBlocks(source).map(render).join(''), render(source))
  }
})
