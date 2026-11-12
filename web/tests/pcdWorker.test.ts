import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'
import * as pcd from '../src/services/pcdPreview.ts'
import * as THREE from 'three'
import * as heightColors from '../src/services/cloudHeightColor.ts'
import { createPcdLayer } from '../src/components/scene3d/layers/pcdLayer.ts'

test('full-cloud worker retains every point beyond the thumbnail limit and converts map axes', () => {
  const count = 32768
  const header = new TextEncoder().encode(`FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH ${count}\nHEIGHT 1\nPOINTS ${count}\nDATA binary\n`)
  const buffer = new ArrayBuffer(header.length + count * 12)
  new Uint8Array(buffer).set(header)
  const source = new DataView(buffer, header.length)
  for (let i = 0; i < count; i++) {
    source.setFloat32(i * 12, i, true)
    source.setFloat32(i * 12 + 4, i % 8, true)
    source.setFloat32(i * 12 + 8, i % 5, true)
  }
  let result: { cloud?: pcd.PcdCloud; error?: string } = {}
  const self = { onmessage: undefined as ((event: { data: ArrayBuffer }) => void) | undefined,
    postMessage(value: typeof result) { result = value } }
  const script = readFileSync(new URL('../src/workers/pcd.worker.ts', import.meta.url), 'utf8')
  runInNewContext(ts.transpileModule(script, { compilerOptions: {
    module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022,
  } }).outputText, { exports: {}, self, require: (name: string) =>
    name.includes('pcdPreview') ? pcd : name.includes('cloudHeightColor') ? heightColors : THREE })
  self.onmessage!({ data: buffer })
  assert.equal(result.error, undefined)
  const cloud = result.cloud!
  assert.equal(cloud.sourceCount, count)
  assert.equal(cloud.positions.length, count * 3)
  assert.deepEqual(Array.from(cloud.positions.slice(-3)), [count - 1, (count - 1) % 8, (count - 1) % 5])
  const layer = createPcdLayer(cloud)
  assert.equal(layer.geometry.getAttribute('position').array, cloud.positions, 'full geometry uses the decoded buffer directly')
  const bounds = new THREE.Box3().setFromObject(layer)
  assert.equal(bounds.max.y, 4, 'map height becomes Three.js Y')
  assert.equal(bounds.min.z, -7)
  assert.equal(layer.geometry.getAttribute('color'), undefined, 'height colors are computed on the GPU')
  assert.ok(layer.geometry.boundingSphere!.containsPoint(new THREE.Vector3(count - 1, 7, 4)))
  layer.geometry.dispose()
  layer.material.dispose()
})
