import assert from 'node:assert/strict'
import test from 'node:test'
import { addModelMarks } from '../src/components/agentConversation/modelMentions.ts'
import { renderMarkdown } from '../src/guide/markdown.ts'

const render = (text: string) => addModelMarks(renderMarkdown(text, { pageId: 'agent', resolveDocumentLink: () => undefined, renderImages: true }))

test('model mentions use retained icons and preserve labels and links', () => {
  const html = render('可以选择 **Claude Opus 5.5**、[Codex](https://github.com/openai/codex)、GPT-6.1、Kimi、Qwen3、Gemini、DeepSeek、Grok、Mistral、Llama、Perplexity、MiniMax、GLM、Cohere 和通义千问。')
  assert.equal((html.match(/class="model-mention"/g) ?? []).length, 15)
  assert.match(html, /anthropic\.png/)
  assert.match(html, /moonshot\.ico/)
  assert.match(html, /qwen\.png/)
  assert.match(html, /google-gemini\.png/)
  assert.match(html, /deepseek\.ico/)
  assert.match(html, /grok\.ico/)
  assert.match(html, /mistral\.ico/)
  assert.match(html, /meta\.ico/)
  assert.match(html, /perplexity\.ico/)
  assert.match(html, /minimax\.ico/)
  assert.match(html, /zhipu\.png/)
  assert.match(html, /cohere\.ico/)
  assert.match(html, /title="DeepSeek · 仅用于识别名称"/)
  assert.match(html, /href="https:\/\/github.com\/openai\/codex"/)
  assert.match(html, />Claude<\/span> Opus 5.5/)
  assert.match(html, />GPT-6.1<\/span>/)
  assert.doesNotMatch(html, /undefined/)
})

test('code paths identifiers URLs and image attributes are not decorated', () => {
  const text = '`Claude`\n\n```text\nCodex OpenAI\n```\n\nhttps://example.com/Codex https://example.com?q=Claude codex.ts /opt/Kimi C:\\Codex\\file.txt myQwen CodexTool a@OpenAI.com\n\n![Claude](https://example.com/Claude.png)'
  assert.doesNotMatch(render(text), /class="model-mention"/)
})

test('HTML remains escaped and provider marks carry no text into copy or selection', () => {
  const html = render('<script>Claude</script> OpenAI')
  assert.doesNotMatch(html, /<script>/)
  assert.match(html, /alt="" aria-hidden="true" draggable="false"/)
  assert.equal(html.replace(/<[^>]*>/g, ''), '&lt;script&gt;Claude&lt;/script&gt; OpenAI')
})
