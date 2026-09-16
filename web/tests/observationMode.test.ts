import assert from 'node:assert/strict'
import test from 'node:test'
import { dashboardFetch, isObservationMode } from '../src/services/observationMode.ts'

test('observation session cannot dispatch navigation, localization, map changes or stop commands', async () => {
  const originalWindow = Object.getOwnPropertyDescriptor(globalThis, 'window')
  const originalFetch = globalThis.fetch
  const received: string[] = []
  Object.defineProperty(globalThis, 'window', { configurable: true, value: { location: { search: '?observe=1' } } })
  globalThis.fetch = async (input) => { received.push(String(input)); return new Response('{}') }
  try {
    for (const [path, method] of [
      ['/api/v1/navigate/click', 'POST'], ['/api/v1/localization/relocalizations', 'POST'],
      ['/api/v1/maps/example', 'DELETE'], ['/api/v1/control', 'POST'],
    ]) await assert.rejects(dashboardFetch(path, { method }), /只读监控/)
    assert.deepEqual(received, [])
    await dashboardFetch('/api/v1/state')
    assert.deepEqual(received, ['/api/v1/state'])
    Object.defineProperty(globalThis, 'window', { configurable: true, value: { location: { search: '' } } })
    await dashboardFetch('/api/v1/control', { method: 'POST' })
    assert.equal(received.length, 2, 'normal operator mode keeps its existing command path')
  } finally {
    globalThis.fetch = originalFetch
    if (originalWindow) Object.defineProperty(globalThis, 'window', originalWindow)
    else Reflect.deleteProperty(globalThis, 'window')
  }
})

test('observation mode is an explicit URL choice', () => {
  assert.equal(isObservationMode('?observe=1&debug_nav=1'), true)
  assert.equal(isObservationMode('?observe=0'), false)
  assert.equal(isObservationMode(''), false)
})
