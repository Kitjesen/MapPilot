import assert from 'node:assert/strict'
import test from 'node:test'
import * as THREE from 'three'
import { fetchSurfaceMap, defaultSurfaceOptions, type SurfaceMap } from '../src/services/surfaceMap.ts'
import { createSupportLayer } from '../src/components/scene3d/layers/supportLayer.ts'

const surface: SurfaceMap = { map_id: 'hall', content_epoch: 42, frame_id: 'map', resolution: .1,
  options: defaultSurfaceOptions, observed_count: 2, inferred_count: 1, inferred_area_m2: .01,
  observed_xyz: [1, 2, 0, 1, 2, 2], inferred_xyz: [1.1, 2, 0], sampled: false }

test('surface query preserves options, complete counts and stacked heights', async () => {
  const previous = globalThis.fetch
  let url = ''
  try {
    globalThis.fetch = async input => { url = String(input); return new Response(JSON.stringify(surface)) }
    const result = await fetchSurfaceMap('hall', defaultSurfaceOptions)
    assert.deepEqual(result, surface)
    assert.match(url, /hall\/surface\?/)
    assert.match(url, /headroom_m=0.6/)
    assert.match(url, /max_hole_area_m2=0.16/)
    for (const invalid of [{ ...surface, map_id: 'other' }, { ...surface, inferred_xyz: [1, 2] }]) {
      globalThis.fetch = async () => new Response(JSON.stringify(invalid))
      await assert.rejects(fetchSurfaceMap('hall', defaultSurfaceOptions), /与当前地图不匹配/)
    }
  } finally { globalThis.fetch = previous }
})

test('observed and inferred support are separate horizontal cells with world XYZ coordinates', () => {
  const layer = createSupportLayer(surface)
  assert.equal(layer.name, 'support-surfaces')
  assert.deepEqual(layer.children.map(object => object.name), ['observed', 'inferred'])
  const observed = layer.children[0] as THREE.InstancedMesh
  const inferred = layer.children[1] as THREE.InstancedMesh
  assert.equal(observed.count, 2)
  assert.equal(inferred.count, 1)
  const matrix = new THREE.Matrix4()
  observed.getMatrixAt(1, matrix)
  const position = new THREE.Vector3().setFromMatrixPosition(matrix)
  assert.deepEqual(position.toArray(), [1, 2, -2])
  assert.notEqual((observed.material as THREE.MeshBasicMaterial).color.getHex(),
    (inferred.material as THREE.MeshBasicMaterial).color.getHex())
  layer.traverse(object => {
    const mesh = object as THREE.Mesh
    mesh.geometry?.dispose()
    if (mesh.material) for (const material of Array.isArray(mesh.material) ? mesh.material : [mesh.material]) material.dispose()
  })
})
