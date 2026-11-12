import { tracePath } from '../src/services/sensorChart.ts'
import assert from 'node:assert/strict'
import test from 'node:test'
import { mergeImu, readImu, imuDownload, type ImuSample } from '../src/services/imu.ts'
const row = (seq: number, t: number, z: number, gap_before = false): ImuSample => ({ seq, t, received_at_s: t, values: { angular_velocity_rad_s: { z } }, gap_before })

test('live data deduplicates and keeps a bounded window across source restarts', () => {
  const result = mergeImu([row(1,0,0), row(2,100,1)], [row(2,100,1), row(3,301,2)])
  assert.deepEqual(result.map(item => item.seq), [2,3])
  assert.equal(mergeImu([row(1,0,0)], [row(9,1,1)])[1].gap_before, true)
  assert.deepEqual(mergeImu(result, [row(1,0,3)], true).map(item => item.seq), [1])
})
test('chart preserves a one-sample spike and breaks at a source gap or missing field', () => {
  const rows = [row(1,0,0),row(2,.001,9),row(3,.002,0),row(4,.003,1,true),row(5,.004,0)]
  const flat = rows.map(row => ({ ...row, values: { z: row.values.angular_velocity_rad_s.z } }))
  const path = tracePath(flat, 'z', 0, 60, 0, 10)
  assert.ok(path.includes('17.6'))
  assert.equal((path.match(/M/g) ?? []).length, 2)
  assert.equal(tracePath(flat, 'missing', 0,60,0,1), '')
})
test('live decoder rejects malformed measurements and download retains exact identity', async () => {
  const previous = globalThis.fetch
  try {
    globalThis.fetch = async () => new Response(JSON.stringify({ samples: [{ ...row(1,1,0), t: null }] }))
    await assert.rejects(readImu(0), /格式无效/)
  } finally { globalThis.fetch = previous }
  assert.equal(imuDownload('imu-example','csv'), '/api/v1/robot/imu/records/imu-example/download?format=csv')
})
