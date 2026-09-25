import assert from 'node:assert/strict'
import test from 'node:test'
import { readFileSync } from 'node:fs'
import {
  fetchProductOperation, submitProductSwitch, switchIsTerminal, ProductControlError,
  type ProductSwitchOperation,
} from '../src/services/productControl.ts'
import { makeRequestId } from '../src/services/api.ts'

const originalFetch = globalThis.fetch
test.afterEach(() => { globalThis.fetch = originalFetch })

test('mode switching uses the dedicated transport and preserves request identity', async () => {
  const request = { request_id: 'product-test', product: 'nav' as const, map_name: 'office', expected_product_session_id: 'old', initial_pose: { x: 1, y: 2, z: 0.35, yaw: 0.4 } }
  globalThis.fetch = async (url, init) => {
    assert.equal(url, '/api/v1/product-control/switch')
    assert.equal(init?.method, 'POST')
    assert.deepEqual(JSON.parse(String(init?.body)), request)
    return Response.json({ request_id: request.request_id, request, state: 'running' }, { status: 202 })
  }
  assert.equal((await submitProductSwitch(request)).state, 'running')
})

test('reconnecting queries the original operation without resubmitting motion', async () => {
  globalThis.fetch = async (url, init) => {
    assert.equal(url, '/api/v1/product-control/operations/product-test')
    assert.equal(init?.method, 'GET')
    return Response.json({ state: 'succeeded', request_id: 'product-test' })
  }
  assert.equal((await fetchProductOperation('product-test')).state, 'succeeded')
})

test('service rejection and interruption are not reported as successful switches', async () => {
  globalThis.fetch = async () => Response.json({ message: '已有切换' }, { status: 409 })
  await assert.rejects(fetchProductOperation('product-test'), error => error instanceof ProductControlError && error.status === 409)
  for (const state of ['succeeded', 'failed', 'interrupted'] as const) {
    assert.equal(switchIsTerminal({ state } as ProductSwitchOperation), true)
  }
  assert.equal(switchIsTerminal({ state: 'running' } as ProductSwitchOperation), false)
})

test('wired HTTP access can create request ids without secure-context randomUUID', () => {
  const descriptor = Object.getOwnPropertyDescriptor(globalThis, 'crypto')
  try {
    Object.defineProperty(globalThis, 'crypto', { configurable: true, value: undefined })
    assert.match(makeRequestId('product'), /^product-[a-z0-9-]+$/)
  } finally {
    if (descriptor) Object.defineProperty(globalThis, 'crypto', descriptor)
    else Reflect.deleteProperty(globalThis, 'crypto')
  }
})

test('mapping and saved-map navigation have distinct controls and keep the preview read-only', () => {
  const scene = readFileSync(new URL('../src/components/SceneView.tsx', import.meta.url), 'utf8')
  const maps = readFileSync(new URL('../src/components/MapView.tsx', import.meta.url), 'utf8')
  assert.match(scene, /onClick=\{onStartMapping\}/)
  assert.match(scene, /累计地图/)
  assert.match(scene, /适应全图/)
  assert.match(maps, /使用此地图导航/)
  assert.match(maps, /最新补扫已保存/)
  assert.match(maps, /if \(name\) onUseMap\(name, pose\)/)
  assert.match(maps, /mapIsActivationReady\(selectedInfo\)/)
})
