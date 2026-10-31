import assert from 'node:assert/strict'
import test from 'node:test'
import { formatCommandError, GatewayApiError, sendInstruction } from '../src/services/api.ts'

const originalFetch = globalThis.fetch
test.afterEach(() => { globalThis.fetch = originalFetch })

for (const status of [502, 503, 504]) {
  test(`an unstructured HTTP ${status} retains uncertain command outcome without retrying`, async () => {
    let requests = 0
    globalThis.fetch = async () => {
      requests++
      return new Response(status === 502 ? '' : '<html>upstream unavailable</html>', { status })
    }
    await assert.rejects(sendInstruction('帮我找一瓶水'), (error: unknown) => {
      assert.ok(error instanceof GatewayApiError)
      assert.equal(error.statusCode, status)
      const message = formatCommandError(error, '指令发送失败')
      assert.match(message, /未收到机器人服务的确认/)
      assert.ok(message.includes(`HTTP ${status}`))
      assert.match(message, /核对执行结果/)
      assert.doesNotMatch(message, /模型|任务未创建|任务已停止/)
      return true
    })
    assert.equal(requests, 1)
  })
}

test('structured service refusal keeps its domain reason', async () => {
  globalThis.fetch = async () => Response.json({
    ok: false, error: 'camera_unavailable', message: '相机标定尚未完成',
  }, { status: 503 })
  await assert.rejects(sendInstruction('帮我找一瓶水'), (error: unknown) => {
    assert.equal(formatCommandError(error, '指令发送失败'), '指令发送失败: 相机标定尚未完成')
    return true
  })
})

test('fetch connection failure gets guidance without hiding unrelated TypeErrors', () => {
  assert.match(formatCommandError(new TypeError('Failed to fetch')), /网络请求未完成/)
  assert.equal(formatCommandError(new TypeError('invalid caller value')), '命令失败: invalid caller value')
})
