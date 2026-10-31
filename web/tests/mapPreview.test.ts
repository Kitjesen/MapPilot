import assert from 'node:assert/strict'
import test from 'node:test'

import { fetchMapPreview } from '../src/services/mapPreview.ts'

const originalFetch = globalThis.fetch

test.afterEach(() => {
  globalThis.fetch = originalFetch
})

function response(headers: Record<string, string>) {
  return new Response(new Uint8Array([1, 2, 3]), { headers })
}

test('saved-map preview requires native artifact identity and does not use HTTP cache', async () => {
  let request: RequestInit | undefined
  globalThis.fetch = async (_input, init) => {
    request = init
    return response({
      'X-Lingtu-Map-Id': 'office',
      'X-Lingtu-Content-Epoch': '7',
      'X-Lingtu-Frame-Id': 'map',
    })
  }

  const preview = await fetchMapPreview('office', 7)
  assert.equal(preview.mapId, 'office')
  assert.equal(preview.contentEpoch, 7)
  assert.equal(preview.frameId, 'map')
  assert.equal(request?.cache, 'no-store')
})

test('saved-map preview rejects another map or content version before exposing bytes', async () => {
  globalThis.fetch = async () => response({
    'X-Lingtu-Map-Id': 'other',
    'X-Lingtu-Content-Epoch': '7',
    'X-Lingtu-Frame-Id': 'map',
  })
  await assert.rejects(fetchMapPreview('office', 7), /已切换/)

  globalThis.fetch = async () => response({
    'X-Lingtu-Map-Id': 'office',
    'X-Lingtu-Content-Epoch': '8',
    'X-Lingtu-Frame-Id': 'map',
  })
  await assert.rejects(fetchMapPreview('office', 7), /已更新/)
})

test('saved-map preview accepts the returned epoch only when the list has no expected epoch', async () => {
  globalThis.fetch = async () => response({
    'X-Lingtu-Map-Id': 'office',
    'X-Lingtu-Content-Epoch': '11',
    'X-Lingtu-Frame-Id': 'map',
  })

  assert.equal((await fetchMapPreview('office')).contentEpoch, 11)
})

test('navigation preview requests the native OctoMap through the same identity-bound stream', async () => {
  let url = ''
  globalThis.fetch = async input => {
    url = String(input)
    return response({ 'X-Lingtu-Map-Id': 'office', 'X-Lingtu-Content-Epoch': '7', 'X-Lingtu-Frame-Id': 'map' })
  }
  const preview = await fetchMapPreview('office', 7, undefined, 'octomap')
  assert.equal(url, '/api/v1/maps/office/octomap')
  assert.equal(preview.contentEpoch, 7)
})

test('saved-map preview rejects responses without identity headers', async () => {
  globalThis.fetch = async () => response({})
  await assert.rejects(fetchMapPreview('office'), /缺少 X-Lingtu-Map-Id/)
})

test('saved-map preview rejects zero and invalid native content versions', async () => {
  for (const epoch of ['0', '-1', 'NaN', '1.5']) {
    globalThis.fetch = async () => response({
      'X-Lingtu-Map-Id': 'office',
      'X-Lingtu-Content-Epoch': epoch,
      'X-Lingtu-Frame-Id': 'map',
    })
    await assert.rejects(fetchMapPreview('office'), /无效的内容版本/)
  }
})
