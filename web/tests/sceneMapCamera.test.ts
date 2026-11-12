import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import ts from 'typescript'
import { pcdCameraDistance } from '../src/services/pcdPreview.ts'

const source = readFileSync(new URL('../src/components/Scene3D.tsx', import.meta.url), 'utf8')
const file = ts.createSourceFile('Scene3D.tsx', source, ts.ScriptTarget.Latest, true, ts.ScriptKind.TSX)
let method: ts.MethodDeclaration | undefined
const find = (node: ts.Node) => {
  if (ts.isMethodDeclaration(node) && node.name.getText(file) === 'fitMap') method = node
  ts.forEachChild(node, find)
}
find(file)
assert.ok(method)
const fit = ts.transpileModule(`({ ${method.getText(file)} }).fitMap`, {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS },
}).outputText

test('fit map contains the full occupied bounds in narrow and wide viewports', () => {
  const bounds = new THREE.Box3(new THREE.Vector3(-19.2, -17.7, -35.6), new THREE.Vector3(49.5, 5, 29.8))
  for (const aspect of [435 / 1167, .1, 1, 2]) {
    const camera = new THREE.PerspectiveCamera(48, aspect, .02, 500)
    const controls = new OrbitControls(camera)
    controls.minDistance = .75
    controls.maxDistance = 120
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(...bounds.getSize(new THREE.Vector3()).toArray()))
    mesh.position.copy(bounds.getCenter(new THREE.Vector3()))
    const fitMap = runInNewContext(fit, { THREE, pcdCameraDistance,
      cameraRef: { current: camera }, controlsRef: { current: controls },
      mappingObservationRef: { current: null }, octomapRef: { current: mesh },
      savedMapRef: { current: null }, liveCloudRef: { current: null } }) as () => void
    fitMap()
    camera.updateMatrixWorld()
    for (const x of [bounds.min.x, bounds.max.x]) for (const y of [bounds.min.y, bounds.max.y]) {
      for (const z of [bounds.min.z, bounds.max.z]) {
        const point = new THREE.Vector3(x, y, z).project(camera)
        assert.ok(Math.abs(point.x) <= 1 && Math.abs(point.y) <= 1 && Math.abs(point.z) <= 1,
          `bounds corner clipped at aspect ${aspect}: ${point.toArray()}`)
      }
    }
    assert.ok(camera.far > camera.position.distanceTo(controls.target) + bounds.getSize(new THREE.Vector3()).length() / 2)
    mesh.geometry.dispose(); (mesh.material as THREE.Material).dispose()
  }
})
