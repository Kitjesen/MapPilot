import assert from 'node:assert/strict'
import test from 'node:test'
import { appendTrace, numericFields, readDisplay, sensorRequest, traceCsv, type SensorTrace } from '../src/services/sensors.ts'

test('numeric cards follow actual joint layouts and preserve missing values', () => {
  for (const count of [12,16]) {
    const fields = numericFields({ values: { joints: Array.from({ length: count }, (_, id) => ({ id, name: `joint${id}`, position_rad: id / 10, velocity_rad_s: null, torque_nm: NaN })) } })
    assert.equal(Object.keys(fields).length, count)
    assert.equal(fields['joint0 · 位置 (rad)'], 0)
    assert.equal(fields[`joint${count - 1} · 位置 (rad)`], (count - 1) / 10)
  }
  assert.deepEqual(numericFields({ values: { volts: [24, null, 0, Infinity] } }), { '通道 0 · V': 24, '通道 2 · V': 0 })
  assert.deepEqual(numericFields({ values: { motors: [{ id: 7, online: false, temperature_c: 42, status_code: 4 }] } }), { '7 · 温度 (°C)': 42 })
})

test('display trace stays bounded and CSV leaves gaps empty with source time intact', () => {
  const row = (t: number, values = {}): SensorTrace => ({ t, receivedAt: 1700000000 + t, source: { seconds: t }, values })
  assert.deepEqual(appendTrace([row(0),row(100)], row(301)).map(item => item.t), [100,301])
  assert.equal(appendTrace(Array.from({ length: 1500 }, (_, i) => row(i / 10)), row(150)).length, 1500)
  const csv = traceCsv([row(0, { 'joint,"0"': 1 }), row(1), row(2, { 'joint,"0"': 0 })])
  const lines = csv.split('\r\n')
  assert.ok(lines[0].includes('"joint,""0"""'))
  assert.ok(lines[1].includes('"{""seconds"":0}"'))
  assert.ok(lines[2].endsWith(',""'))
  assert.ok(lines[3].endsWith(',"0"'))
})

test('display preferences accept only supported choices and tolerate unavailable storage', () => {
  const previous = Object.getOwnPropertyDescriptor(globalThis, 'localStorage')
  try {
    Object.defineProperty(globalThis, 'localStorage', { configurable: true, value: { getItem: () => '{"interval":500,"window":300,"pointSize":3}' } })
    assert.deepEqual(readDisplay(), { interval: 500, window: 300, pointSize: 3 })
    Object.defineProperty(globalThis, 'localStorage', { configurable: true, value: { getItem: () => '{"interval":0,"window":999,"pointSize":-1}' } })
    assert.deepEqual(readDisplay(), { interval: 1000, window: 60, pointSize: 2 })
    Object.defineProperty(globalThis, 'localStorage', { configurable: true, value: { getItem: () => { throw new Error('blocked') } } })
    assert.deepEqual(readDisplay(), { interval: 1000, window: 60, pointSize: 2 })
  } finally {
    if (previous) Object.defineProperty(globalThis, 'localStorage', previous)
    else Reflect.deleteProperty(globalThis, 'localStorage')
  }
})

test('sensor reads retain source failures instead of reporting successful empty data', async () => {
  const previous = globalThis.fetch
  try {
    globalThis.fetch = async () => new Response(JSON.stringify({ detail: { success: false, error: 'joints: adapter not configured' } }), { status: 503 })
    await assert.rejects(sensorRequest('/api/v1/robot/sensors/joints', new AbortController().signal), /adapter not configured/)
    globalThis.fetch = async () => new Response(JSON.stringify({ success: false, error: 'source disconnected' }))
    await assert.rejects(sensorRequest('/api/v1/robot/sensors/motors', new AbortController().signal), /source disconnected/)
    globalThis.fetch = async () => new Response(JSON.stringify({ success: true, fresh: false, values: { volts: [24] } }))
    assert.equal((await sensorRequest('/api/v1/robot/sensors/voltages', new AbortController().signal)).fresh, false)
  } finally { globalThis.fetch = previous }
})
