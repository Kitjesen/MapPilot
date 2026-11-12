import assert from 'node:assert/strict'
import test from 'node:test'
import { createAgentPreviewFetch } from './fixtures/agentCapabilitiesPreview.ts'

test('preview is isolated and authorization only succeeds on the second login', async () => {
  const fetch = createAgentPreviewFetch()
  const options = await (await fetch('/api/v1/agent/options')).json()
  assert.deepEqual(options.models.map((model: { provider: string }) => model.provider).sort(), ['claude', 'codex', 'moonshot', 'openai', 'qwen'])
  const path = '/api/v1/agent/connections/codex'
  const first = await (await fetch(`${path}/login`, { method: 'POST' })).json()
  for (let i = 0; i < 4; i++) assert.equal((await (await fetch(`${path}/login/${first.id}/poll`, { method: 'POST' })).json()).state, 'pending')
  assert.equal((await (await fetch(`${path}/login/${first.id}`, { method: 'DELETE' })).json()).state, 'cancelled')
  const second = await (await fetch(`${path}/login`, { method: 'POST' })).json()
  for (let i = 0; i < 3; i++) await fetch(`${path}/login/${second.id}/poll`, { method: 'POST' })
  assert.equal((await (await fetch(path)).json()).connected, true)
  assert.equal((await (await fetch(path)).json()).inference_verified, false)
  assert.equal((await fetch('https://example.com/unexpected')).status, 400)
})

test('preview updates the existing tool message and visibly labels synthetic sources', async () => {
  const fetch = createAgentPreviewFetch()
  const path = '/api/v1/agent/conversations/demo'
  await fetch(`${path}/messages`, { method: 'POST', body: JSON.stringify({ message_id: 'one', text: '搜索 Hermes' }) })
  const pending = await (await fetch(path)).json()
  assert.equal(pending.turns[0].state, 'running')
  const toolId = pending.messages[1].id
  await new Promise(resolve => setTimeout(resolve, 2300))
  const done = await (await fetch(path)).json()
  assert.equal(done.turns[0].state, 'completed')
  assert.equal(done.messages[1].id, toolId)
  assert.match(JSON.parse(done.messages[1].text).sources[0].content, /演示摘要/)
  assert.ok(done.messages[1].metadata.duration_ms >= 2200)
})
