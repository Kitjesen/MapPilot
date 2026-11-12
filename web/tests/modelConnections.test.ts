import assert from 'node:assert/strict'
import test from 'node:test'
import { cancelCodexLogin, disconnectCodex, getCodexConnection, pollCodexLogin, startCodexLogin } from '../src/services/modelConnections.ts'

test('model connections use scoped authorization calls without exposing tokens', async () => {
  const calls: Array<[string, string]> = []
  const original = globalThis.fetch
  const connection = { provider: 'codex', connected: false, inference_verified: false }
  const login = { id: 'login-one', state: 'pending', user_code: 'TEST-CODE', verification_url: 'https://auth.openai.com/codex/device', expires_at: 2000, interval: 5 }
  globalThis.fetch = async (input, init) => {
    calls.push([String(input), init?.method ?? 'GET'])
    return Response.json(String(input).includes('/login') ? login : connection)
  }
  try {
    assert.equal((await getCodexConnection()).connected, false)
    assert.equal((await startCodexLogin()).id, 'login-one')
    await pollCodexLogin('login-one')
    await cancelCodexLogin('login-one')
    await disconnectCodex()
    assert.deepEqual(calls.map(row => row[1]), ['GET', 'POST', 'POST', 'DELETE', 'DELETE'])
    assert.match(calls[2][0], /login\/login-one\/poll$/)
  } finally { globalThis.fetch = original }
})

test('authorization cannot link to a substituted provider page', async () => {
  const original = globalThis.fetch
  globalThis.fetch = async () => Response.json({ id: 'x', state: 'pending', user_code: 'code', expires_at: 10, interval: 5, verification_url: 'https://example.com/login' })
  try { await assert.rejects(startCodexLogin, /登录状态无效/) }
  finally { globalThis.fetch = original }
})
