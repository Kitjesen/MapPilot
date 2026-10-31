import assert from 'node:assert/strict'
import test from 'node:test'
import { readFileSync } from 'node:fs'
import { saveEditedMap, buildMapOctomap } from '../src/services/api.ts'

for (const resolution of [.06, .1]) {
  test(`resolution workflow copies the source and rebuilds it at ${resolution * 100} cm`, async () => {
    const originalFetch = globalThis.fetch
    const calls: Array<{ url: string; body: unknown }> = []
    try {
      globalThis.fetch = async (input, init) => {
        calls.push({ url: String(input), body: JSON.parse(String(init?.body)) })
        return new Response(JSON.stringify(calls.length === 1
          ? { ok: true, saved_rays_preserved: true, build_options: { frame_id: 'floor_903', resolution: .05 } }
          : { ok: true, octomap_ok: true, resolution }))
      }
      const copied = await saveEditedMap({ name: 'original', contentEpoch: 11 }, 'coarse', [])
      const built = await buildMapOctomap('coarse', { ...copied.build_options, resolution })
      assert.equal(built.resolution, resolution)
      assert.deepEqual(calls, [
        { url: '/api/v1/maps/original/save_as', body: { name: 'coarse', expected_source_content_epoch: 11, operations: [] } },
        { url: '/api/v1/maps/coarse/build_octomap', body: { frame_id: 'floor_903', resolution } },
      ])
    } finally { globalThis.fetch = originalFetch }
  })
}

test('scene links display aggregation to actual map rebuilding and activation stays explicit', () => {
  const scene = readFileSync(new URL('../src/components/SceneView.tsx', import.meta.url), 'utf8')
  const maps = readFileSync(new URL('../src/components/MapView.tsx', import.meta.url), 'utf8')
  assert.match(scene, /onOpenSavedMap\(activeMapName \?\? null\).*重建实际导航分辨率/)
  assert.match(maps, /<option value=\{0\.06\}>6 cm<\/option>/)
  assert.match(maps, /另存为此分辨率导航地图/)
  assert.match(maps, /api\.saveEditedMap\(.+, name, \[\]\)/)
  assert.match(maps, /api\.buildMapOctomap\(target\.name, target\.options\)/)
  assert.match(maps, /使用此地图导航后生效/)
})
