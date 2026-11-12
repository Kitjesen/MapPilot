import assert from 'node:assert/strict'
import test from 'node:test'
import { fetchMapThumbnail, mapCleanupLabel, mapUpdateTime, thumbnailProjection } from '../src/services/mapThumbnail.ts'

const nativeFetch = globalThis.fetch
test.afterEach(() => { globalThis.fetch = nativeFetch })
const sample = { name: 'garage', content_epoch: 7, frame_id: 'map', count: 2, total_points: 500,
  points: [[-1, 2, 0], [2, 3, 1]], built_at_ms: 1794211200000, cleanup_state: 'unknown' }

test('thumbnail reads a versioned preview without activating a map', async () => {
  const urls: string[] = []
  globalThis.fetch = async (url, init) => { urls.push(String(url)); assert.equal(init?.method, undefined); return Response.json(sample) }
  const result = await fetchMapThumbnail('garage', 7)
  assert.deepEqual(urls, ['/api/v1/maps/garage/preview?content_epoch=7'])
  assert.equal(result.total_points, 500)
  assert.equal(result.count, 2)
})

test('wrong map, replaced content and malformed coordinates cannot become a thumbnail', async () => {
  for (const body of [{ ...sample, name: 'other' }, { ...sample, content_epoch: 8 },
    { ...sample, points: [[1, 2]] }, { ...sample, points: [[1, null, 3]] }]) {
    globalThis.fetch = async () => Response.json(body)
    await assert.rejects(fetchMapThumbnail('garage', 7))
  }
  globalThis.fetch = async () => Response.json({}, { status: 409 })
  await assert.rejects(fetchMapThumbnail('garage', 7), /HTTP 409/)
})

test('projection preserves height and fits all preview points inside the image', () => {
  const points: Array<[number, number, number]> = [[-8, 2, 0], [5, -4, 1], [0, 0, 0], [0, 0, 3]]
  const image = thumbnailProjection(points, 640)
  assert.equal(image.length, points.length)
  for (const point of image) for (const value of point) assert.ok(value >= 0 && value <= 640)
  assert.equal(image[2][0], image[3][0])
  assert.ok(image[3][1] < image[2][1])
  assert.deepEqual(thumbnailProjection([], 640), [])
})

test('invalid preview counts and frame metadata are rejected before rendering', async () => {
  for (const body of [{ ...sample, count: 3 }, { ...sample, total_points: 1 }, { ...sample, frame_id: 123 }]) {
    globalThis.fetch = async () => Response.json(body)
    await assert.rejects(fetchMapThumbnail('garage', 7), /元数据无效/)
  }
})

test('missing cleaning and time evidence stays unknown', () => {
  assert.equal(mapCleanupLabel(undefined), '清洗状态未知')
  assert.equal(mapCleanupLabel('unknown'), '清洗状态未知')
  assert.equal(mapCleanupLabel('cleaned'), '已清洗')
  assert.equal(mapCleanupLabel('not_cleaned'), '未清洗')
  assert.equal(mapUpdateTime(null), '未记录')
  assert.match(mapUpdateTime(1794211200000), /2026/)
})
