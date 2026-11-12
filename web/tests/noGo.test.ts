import assert from 'node:assert/strict'
import test from 'node:test'
import { fetchNoGo, putNoGo, deleteNoGo } from '../src/services/noGo.ts'
import { createNoGoLayer } from '../src/components/scene3d/layers/noGoLayer.ts'
import * as THREE from 'three'

const region = { id: 'door', min: [1, 2, 3] as [number, number, number], max: [4, 6, 8] as [number, number, number] }
const saved = { map_id: 'office', content_epoch: 42, frame_id: 'map', regions: [region] }

test('no-go requests retain map identity and mutation epoch without editing occupancy', async () => {
  const previous = globalThis.fetch
  const calls: Array<{ path: string; init?: RequestInit }> = []
  try {
    globalThis.fetch = async (input, init) => {
      calls.push({ path: String(input), init })
      return new Response(JSON.stringify(saved))
    }
    assert.deepEqual(await fetchNoGo('office'), saved)
    await putNoGo('office', 42, region)
    await deleteNoGo('office', 43, 'door')
    assert.equal(calls[0].path, '/api/v1/maps/office/no_go')
    assert.equal(calls[1].init?.method, 'PUT')
    assert.deepEqual(JSON.parse(String(calls[1].init?.body)), { expected_content_epoch: 42, region })
    assert.equal(calls[2].path, '/api/v1/maps/office/no_go/door?expected_content_epoch=43')
    assert.equal(calls[2].init?.method, 'DELETE')
    assert.ok(calls.every(call => !call.path.includes('mark_zone') && !call.path.includes('voxels')))
  } finally { globalThis.fetch = previous }
})

test('no-go reads reject mismatched identities and invalid 3D volumes, and keep service errors', async () => {
  const previous = globalThis.fetch
  try {
    for (const value of [{ ...saved, map_id: 'other' }, { ...saved, content_epoch: 0 },
      { ...saved, regions: [{ ...region, min: [1, 2, 9] }] }]) {
      globalThis.fetch = async () => new Response(JSON.stringify(value))
      await assert.rejects(fetchNoGo('office'), /与当前地图不匹配/)
    }
    globalThis.fetch = async () => new Response(JSON.stringify({ success: false, message: '地图正在使用' }), { status: 409 })
    await assert.rejects(putNoGo('office', 42, region), /地图正在使用/)
  } finally { globalThis.fetch = previous }
})

test('no-go display is an independent finite 3D volume with correct world axes', () => {
  const layer = createNoGoLayer([region])
  assert.equal(layer.name, 'no-go-regions')
  const mesh = layer.children[0] as THREE.Mesh<THREE.BoxGeometry, THREE.MeshBasicMaterial>
  assert.equal(mesh.name, 'door')
  assert.deepEqual(mesh.position.toArray(), [2.5, 5.5, -4])
  assert.deepEqual([mesh.geometry.parameters.width, mesh.geometry.parameters.height, mesh.geometry.parameters.depth], [3, 5, 4])
  assert.ok(mesh.material.transparent && mesh.material.opacity > 0 && mesh.material.opacity < 1)
  assert.deepEqual(region.min, [1, 2, 3])
  layer.traverse(object => {
    const drawable = object as THREE.Mesh
    drawable.geometry?.dispose()
    if (drawable.material) for (const material of Array.isArray(drawable.material) ? drawable.material : [drawable.material]) material.dispose()
  })
})
