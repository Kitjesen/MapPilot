import assert from 'node:assert/strict'
import test from 'node:test'
import {
  acousticBin, bindingKey, controlDisabledReason, controlMeasurement, getMeasurementPreview,
  hasFreshMeasurement, listMeasurements, MeasurementApiError, measurementControlReducer, measurementNoise,
  pollMeasurements, semanticValue,
  thermalValue, validateControlParams,
  type AcousticDisplay, type ControlAck, type MeasurementStatus, type SemanticDisplay, type ThermalDisplay,
} from '../src/services/measurements.ts'

const status: MeasurementStatus = {
  sensor_id: 'thermal front', status: 'live', fresh: true, stale: false,
  receipt_age_s: .1, stale_timeout_s: 1, modality: 'thermal', controls: null,
  raw_url: '/api/v1/measurements/thermal%20front/raw',
  source_identity: { product_session_id: 'product-1', session_id: 'source-1', model_generation: 1, reset_generation: 2 },
}
const thermal: ThermalDisplay = {
  kind: 'thermal', width: 3, height: 1, source_width: 30, source_height: 10,
  row_indices: [0], column_indices: [0, 14, 29],
  temperature_k: [[273.15, 999, null]], valid_mask: [[true, false, true]],
}
const semantic: SemanticDisplay = {
  ...thermal, kind: 'semantic', ground_truth: true,
  class_ids: [[0, 4, null]], instance_ids: [[0, 13, null]],
}
const acoustic: AcousticDisplay = {
  kind: 'acoustic', sample_rate_hz: 100, source_sample_count: 20,
  bin_start_samples: [0, 10], bin_end_samples: [10, 20],
  pressure_min_pa: [[-2, null], [-1, 0]], pressure_max_pa: [[3, null], [1, 0]],
  valid_fraction: [[.5, 0], [1, 1]],
}

// Relevant fields retained from build/inspection-tests/20261022/sensor-control-s07ezbq8/result.json.
// This is recorded controlled test evidence, not a live/factory Product declaration.
const recordedQuery: ControlAck = {
  request_id: 'query', product_session_id: 'sensor-control-native', session_id: 'compiled-source',
  sensor_id: 'robot.thermal', model_generation: 2, reset_generation: 1,
  applied: true, reason: 'applied',
  settings: {
    enabled: true, rate_hz: 10, recording: false, noise_enabled: false,
    noise: { unit: 'K', enabled: false, stddev: 0, seed: 0 },
    measurement_settings: {
      noise: { unit: 'K', enabled: false, stddev: 0, seed: 0 },
      camera: { gimbal: { pan_deg: 0, tilt_deg: 0 } },
    },
  },
}
const recordedSensor: MeasurementStatus = {
  ...status, sensor_id: recordedQuery.sensor_id,
  source_identity: {
    product_session_id: recordedQuery.product_session_id, session_id: recordedQuery.session_id,
    model_generation: recordedQuery.model_generation, reset_generation: recordedQuery.reset_generation,
  },
}

test('recorded query ACK exposes nested noise and camera gimbal without invented defaults', () => {
  assert.deepEqual(measurementNoise(recordedQuery.settings), { unit: 'K', enabled: false, stddev: 0, seed: 0 })
  assert.deepEqual(recordedQuery.settings.measurement_settings?.camera?.gimbal, { pan_deg: 0, tilt_deg: 0 })
  assert.deepEqual(recordedQuery.settings.noise, recordedQuery.settings.measurement_settings?.noise)
  assert.equal(measurementNoise(null), null)
})
test('nested measurement noise is supported without a top-level alias; top-level session noise remains supported', () => {
  const noise = { unit: 'Pa', enabled: true, stddev: .5, seed: 7 }
  assert.equal(measurementNoise({ measurement_settings: { noise } }), noise)
  assert.equal(measurementNoise({ noise }), noise)
  assert.equal(measurementNoise({ noise: recordedQuery.settings.noise, measurement_settings: { noise } }), noise)
})
test('query ACK populates current settings; later operator status replaces every setting instead of caching ACK forever', () => {
  const initial = { sensors: [recordedSensor], controlRevision: 0 }
  const queried = measurementControlReducer(initial, { type: 'ack', ack: recordedQuery })
  assert.equal(initial.sensors[0].controls, null)
  assert.equal(queried.sensors[0].controls, recordedQuery.settings)
  const external = { ...recordedSensor, controls: {
    ...recordedQuery.settings, rate_hz: 12, enabled: false, recording: true, noise_enabled: true,
    measurement_settings: { noise: { unit: 'K', enabled: true, stddev: .5, seed: 7 },
      camera: { gimbal: { pan_deg: 10, tilt_deg: -5 } } },
  } }
  const refreshed = measurementControlReducer(queried, { type: 'list', sensors: [external], startedAtRevision: 1 })
  assert.equal(refreshed.sensors[0].controls, external.controls)
  assert.equal(measurementNoise(refreshed.sensors[0].controls)?.seed, 7)
  assert.equal(refreshed.sensors[0].controls?.measurement_settings?.camera?.gimbal?.pan_deg, 10)
})
test('in-flight GET from before ACK cannot roll settings back; the next GET remains authoritative', () => {
  const initial = { sensors: [recordedSensor], controlRevision: 0 }
  const confirmed = measurementControlReducer(initial, { type: 'ack', ack: recordedQuery })
  const oldRead = measurementControlReducer(confirmed, { type: 'list', sensors: [recordedSensor], startedAtRevision: 0 })
  assert.equal(oldRead, confirmed)
  const nextRead = measurementControlReducer(oldRead, { type: 'list', sensors: [recordedSensor], startedAtRevision: 1 })
  assert.equal(nextRead.sensors[0].controls, null)
})
test('rejected ACK does not change confirmed settings; an old binding ACK cannot replace a new session', () => {
  const initial = { sensors: [recordedSensor], controlRevision: 0 }
  assert.equal(measurementControlReducer(initial, { type: 'ack', ack: { ...recordedQuery, applied: false } }), initial)
  const reset = { ...recordedSensor, source_identity: { ...recordedSensor.source_identity!, reset_generation: 99 } }
  const updated = measurementControlReducer({ sensors: [reset], controlRevision: 0 }, { type: 'ack', ack: recordedQuery })
  assert.equal(updated.sensors[0], reset)
  assert.equal(updated.sensors[0].controls, null)
})
test('query is a settings-only intent with empty params, not a motion command', async t => {
  t.mock.method(globalThis, 'fetch', async (url: string, init: RequestInit) => {
    assert.equal(url, '/api/v1/sensors/robot.thermal/control')
    const body = JSON.parse(init.body as string)
    assert.equal(body.operation, 'query')
    assert.deepEqual(body.params, {})
    assert.deepEqual(Object.keys(body).sort(), ['expected_product_session_id', 'operation', 'params', 'request_id'])
    return Response.json({ ...recordedQuery, request_id: body.request_id })
  })
  const result = await controlMeasurement(recordedSensor, 'query', {})
  assert.deepEqual(result.settings, recordedQuery.settings)
})

test('thermal K/C conversion preserves real zero and rejects null, masked and nonfinite pixels', () => {
  assert.equal(thermalValue(thermal, 0, 0, 'C'), 0)
  assert.equal(thermalValue(thermal, 0, 0, 'K'), 273.15)
  assert.equal(thermalValue(thermal, 0, 1, 'K'), null)
  assert.equal(thermalValue(thermal, 0, 2, 'C'), null)
  assert.equal(thermalValue({ ...thermal, temperature_k: [[NaN, Infinity]] }, 0, 0, 'K'), null)
})
test('semantic values are integer class/instance ground truth, including zero, never detections', () => {
  assert.deepEqual(semanticValue(semantic, 0, 0), { classId: 0, instanceId: 0 })
  assert.equal(semanticValue(semantic, 0, 1), null)
  assert.equal(semanticValue(semantic, 0, 2), null)
  assert.equal(semanticValue({ ...semantic, class_ids: [[1.5]] }, 0, 0), null)
  assert.equal(semanticValue({ ...semantic, ground_truth: false } as unknown as SemanticDisplay, 0, 0), null)
})
test('acoustic envelope uses Pa, all channels, exclusive sample bounds and partial validity', () => {
  assert.deepEqual(acousticBin(acoustic, 0, 0), { min: -2, max: 3, fraction: .5, start: 0, end: .1 })
  assert.equal(acousticBin(acoustic, 0, 1), null)
  assert.deepEqual(acousticBin(acoustic, 1, 1), { min: 0, max: 0, fraction: 1, start: .1, end: .2 })
})
test('freshness fails closed for stale/disabled/clock-invalid and elapsed receipt age', () => {
  assert.equal(hasFreshMeasurement(status), true)
  for (const state of ['stale', 'disabled', 'invalid_clock', 'no_sample']) {
    assert.equal(hasFreshMeasurement({ ...status, status: state }), false)
  }
  for (const receipt_age_s of [null, 1, 2, NaN]) assert.equal(hasFreshMeasurement({ ...status, receipt_age_s }), false)
  assert.equal(hasFreshMeasurement({ ...status, stale: true }), false)
})
test('missing session and offline controls carry explicit reasons; reset changes binding', () => {
  assert.match(controlDisabledReason({ ...status, source_identity: null }), /No source session binding/)
  assert.match(controlDisabledReason(status, false), /Offline/)
  assert.equal(controlDisabledReason(status), '')
  assert.notEqual(bindingKey(status), bindingKey({ ...status, source_identity: { ...status.source_identity!, reset_generation: 3 } }))
})
test('noise seeds must be nonnegative safe integers; units and numeric params are explicit', () => {
  for (const seed of [-1, .5, Number.MAX_SAFE_INTEGER + 1, Infinity, '1']) {
    assert.throws(() => validateControlParams('noise', { enabled: true, unit: 'K', stddev: .2, seed }), /safe integer/)
  }
  validateControlParams('noise', { enabled: false, unit: 'Pa', stddev: 0, seed: Number.MAX_SAFE_INTEGER })
  assert.throws(() => validateControlParams('noise', { enabled: true, unit: '', stddev: 1, seed: 1 }), /unit/)
  assert.throws(() => validateControlParams('rate', { rate_hz: 0 }), /positive/)
  assert.throws(() => validateControlParams('gimbal', { pan_deg: NaN, tilt_deg: 0 }), /finite/)
  assert.throws(() => validateControlParams('enabled', { enabled: 'false' }), /boolean/)
})
test('GET contracts encode sensor IDs, never request raw data, and disable caching', async t => {
  const urls: string[] = []
  t.mock.method(globalThis, 'fetch', async (input: string, init: RequestInit) => {
    urls.push(input); assert.equal(init.cache, 'no-store')
    return Response.json(input.endsWith('/preview') ? { measurement: status, display: thermal } : { sensors: [status] })
  })
  assert.deepEqual((await listMeasurements()).sensors, [status])
  assert.deepEqual((await getMeasurementPreview('thermal front')).display, thermal)
  assert.deepEqual(urls, ['/api/v1/measurements', '/api/v1/measurements/thermal%20front/preview'])
})
for (const code of [404, 409, 503]) test(`preview HTTP ${code} is unavailable, not substituted display data`, async t => {
  t.mock.method(globalThis, 'fetch', async () => Response.json({ error: 'unavailable' }, { status: code }))
  await assert.rejects(getMeasurementPreview('sensor'), error => error instanceof MeasurementApiError && error.status === code)
})
test('control sends UUID, bound product and exact params; displays actual ACK settings, not requested values', async t => {
  t.mock.method(globalThis, 'fetch', async (url: string, init: RequestInit) => {
    assert.equal(url, '/api/v1/sensors/thermal%20front/control')
    const body = JSON.parse(init.body as string)
    assert.equal(init.method, 'POST')
    assert.match(body.request_id, /^[0-9a-f-]{36}$/)
    assert.equal(body.expected_product_session_id, 'product-1')
    assert.deepEqual(body.params, { rate_hz: 10 })
    return Response.json({ ...status.source_identity, sensor_id: status.sensor_id, request_id: body.request_id,
      applied: true, reason: 'applied', settings: { rate_hz: 8 } })
  })
  const ack = await controlMeasurement(status, 'rate', { rate_hz: 10 })
  assert.equal(ack.settings.rate_hz, 8)
  assert.equal(status.controls, null)
})
test('rejected source ACK is returned as rejection, not an applied state', async t => {
  t.mock.method(globalThis, 'fetch', async (_url: string, init: RequestInit) => {
    const body = JSON.parse(init.body as string)
    return Response.json({ ...status.source_identity, sensor_id: status.sensor_id, request_id: body.request_id,
      applied: false, reason: 'unsupported_operation', settings: {} }, { status: 409 })
  })
  assert.equal((await controlMeasurement(status, 'gimbal', { pan_deg: 0, tilt_deg: 0 })).applied, false)
})
test('ACK with wrong binding or request cannot confirm state', async t => {
  t.mock.method(globalThis, 'fetch', async () => Response.json({ ...status.source_identity,
    sensor_id: status.sensor_id, request_id: 'other-request', applied: true, settings: { enabled: false } }))
  await assert.rejects(controlMeasurement(status, 'enabled', { enabled: false }), /identity mismatch/)
})
test('unbound controls never send a request', async t => {
  const mock = t.mock.method(globalThis, 'fetch', async () => { throw new Error('must not send') })
  await assert.rejects(controlMeasurement({ ...status, source_identity: null }, 'query', {}), /binding/)
  assert.equal(mock.mock.callCount(), 0)
})
test('timeout is unconfirmed, with one POST and no automatic resend', async t => {
  const mock = t.mock.method(globalThis, 'fetch', async () => Response.json({ error: 'sensor_control_timeout' }, { status: 504 }))
  await assert.rejects(controlMeasurement(status, 'query', {}), /sensor_control_timeout/)
  assert.equal(mock.mock.callCount(), 1)
})
test('request timeout aborts the read and cleans its timer', async t => {
  t.mock.timers.enable({ apis: ['setTimeout'] })
  let signal: AbortSignal | undefined
  t.mock.method(globalThis, 'fetch', (_url: string, init: RequestInit) => new Promise((_resolve, reject) => {
    signal = init.signal!; signal.addEventListener('abort', () => reject(new Error('aborted')), { once: true })
  }))
  const pending = listMeasurements()
  t.mock.timers.tick(5000)
  await assert.rejects(pending, /aborted/)
  assert.equal(signal?.aborted, true)
})
test('polling serializes reads and aborts/cleans up without reporting teardown as an error', async t => {
  t.mock.timers.enable({ apis: ['setTimeout'] })
  let calls = 0
  let pollSignal: AbortSignal | undefined
  let resolve!: () => void
  const stop = pollMeasurements(signal => {
    calls++; pollSignal = signal
    return new Promise<void>(done => { resolve = done })
  }, () => assert.fail('teardown is not an error'))
  t.mock.timers.tick(10000)
  assert.equal(calls, 1)
  resolve(); await Promise.resolve(); await Promise.resolve()
  t.mock.timers.tick(1000)
  assert.equal(calls, 2)
  stop(); assert.equal(pollSignal?.aborted, true)
  resolve(); await Promise.resolve(); t.mock.timers.tick(10000)
  assert.equal(calls, 2)
})
test('ACK from another source reset cannot confirm current settings', async t => {
  t.mock.method(globalThis, 'fetch', async (_url: string, init: RequestInit) => {
    const body = JSON.parse(init.body as string)
    return Response.json({ ...status.source_identity, reset_generation: 999,
      sensor_id: status.sensor_id, request_id: body.request_id, applied: true, settings: { enabled: true } })
  })
  await assert.rejects(controlMeasurement(status, 'query', {}), /identity mismatch/)
})
test('caller cancellation propagates to fetch without retry', async t => {
  let fetchSignal: AbortSignal | undefined
  const mock = t.mock.method(globalThis, 'fetch', (_url: string, init: RequestInit) => new Promise((_resolve, reject) => {
    fetchSignal = init.signal!; fetchSignal.addEventListener('abort', () => reject(new Error('cancelled')), { once: true })
  }))
  const controller = new AbortController()
  const pending = getMeasurementPreview('sensor', controller.signal)
  controller.abort()
  await assert.rejects(pending, /cancelled/)
  assert.equal(fetchSignal?.aborted, true)
  assert.equal(mock.mock.callCount(), 1)
})

test('a successful preview that expires in transit is not fresh on arrival', async t => {
  let now = 0
  t.mock.method(performance, 'now', () => now)
  t.mock.method(globalThis, 'fetch', async () => {
    now = 1240
    return Response.json({ measurement: { ...status, receipt_age_s: .1, stale_timeout_s: 1 }, display: {} })
  })
  const result = await getMeasurementPreview(status.sensor_id)
  assert.equal(hasFreshMeasurement(result.measurement), false)
  assert.equal(result.measurement.receipt_age_s, 1.34)
})
test('read polling reports an outage and resumes only after its bounded interval', async t => {
  t.mock.timers.enable({ apis: ['setTimeout'] })
  let calls = 0
  let failures = 0
  const stop = pollMeasurements(async () => {
    if (++calls === 1) throw new Error('offline')
  }, () => { failures++ })
  await Promise.resolve(); await Promise.resolve()
  assert.equal(failures, 1)
  t.mock.timers.tick(999)
  assert.equal(calls, 1)
  t.mock.timers.tick(1)
  assert.equal(calls, 2)
  stop()
})


test('measurement discovery preserves the source failure reason from FastAPI detail', async () => {
  const previous = globalThis.fetch
  try {
    globalThis.fetch = async () => new Response(JSON.stringify({ detail: 'source adapter unavailable' }), { status: 503 })
    await assert.rejects(listMeasurements(), /source adapter unavailable/)
    globalThis.fetch = async () => new Response(JSON.stringify({ detail: { reason: 'source disconnected', sensor_id: 'thermal' } }), { status: 503 })
    await assert.rejects(listMeasurements(), /source disconnected/)
  } finally { globalThis.fetch = previous }
})
