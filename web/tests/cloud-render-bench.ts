import * as THREE from 'three'
import { createPcdLayer } from '../src/components/scene3d/layers/pcdLayer.ts'
import { parsePcd, pcdCameraDistance, type PcdCloud } from '../src/services/pcdPreview.ts'
import { cloudColorRange, CLOUD_HEIGHT_GLSL } from '../src/services/cloudHeightColor.ts'

const status = document.querySelector<HTMLPreElement>('#status')!
const powerPreference = new URLSearchParams(location.search).get('power') === 'high-performance'
  ? 'high-performance' : undefined
const result: Record<string, unknown> = {
  date: new Date().toISOString(), width: 800, height: 600, pixelRatio: 1,
  map: 'test6_cleaned', pointSize: 2,
  revision: 2,
  requestedPowerPreference: powerPreference ?? 'default',
  userAgent: navigator.userAgent,
  visibilityState: document.visibilityState,
  method: 'Interleaved views; CPU render plus synchronous 1-pixel readback, and GPU elapsed timer when available; not interactive FPS',
}
const assert = (condition: unknown, message: string) => { if (!condition) throw new Error(message) }
const legacyVertex = `
#include <clipping_planes_pars_vertex>
uniform float pointSize;
varying float mapHeight;
void main() {
  mapHeight = position.z;
  vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
  gl_Position = projectionMatrix * mvPosition;
  gl_PointSize = pointSize;
  #include <clipping_planes_vertex>
}`
const legacyFragment = `
#include <clipping_planes_pars_fragment>
uniform float opacity;
uniform vec2 heightRange;
varying float mapHeight;
${CLOUD_HEIGHT_GLSL}
void main() {
  #include <clipping_planes_fragment>
  float ratio = heightRange.y > heightRange.x
    ? (mapHeight - heightRange.x) / (heightRange.y - heightRange.x) : 0.5;
  gl_FragColor = vec4(cloudHeightColor(ratio), opacity);
}`

async function run() {
  const response = await fetch('/api/v1/maps/test6_cleaned/pcd', { cache: 'no-store' })
  assert(response.ok, `PCD HTTP ${response.status}`)
  const positions = parsePcd(await response.arrayBuffer())
  assert(positions?.length, 'No decoded points')
  const bounds = new THREE.Box3().setFromBufferAttribute(new THREE.BufferAttribute(positions!, 3))
  const cloud: PcdCloud = { positions: positions!, bounds: { min: bounds.min.toArray(), max: bounds.max.toArray() },
    colorRange: cloudColorRange(positions!)!, sourceCount: positions!.length / 3 }
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true,
    ...(powerPreference ? { powerPreference } : {}) })
  renderer.setPixelRatio(1)
  renderer.setSize(800, 600)
  renderer.setClearColor(0xffffff, 1)
  renderer.localClippingEnabled = true
  document.body.append(renderer.domElement)
  const gl = renderer.getContext()
  result.contextAttributes = gl.getContextAttributes()
  const timer = gl.getExtension('EXT_disjoint_timer_query_webgl2')
  const gl2 = gl as WebGL2RenderingContext
  result.gpuTimerAvailable = Boolean(timer)
  const shaderErrors: string[] = []
  renderer.debug.onShaderError = (context, program, vertex, fragment) => {
    shaderErrors.push([context.getProgramInfoLog(program), context.getShaderInfoLog(vertex), context.getShaderInfoLog(fragment)].join('\n'))
  }
  const debug = gl.getExtension('WEBGL_debug_renderer_info')
  result.gpu = { vendor: debug ? gl.getParameter(debug.UNMASKED_VENDOR_WEBGL) : gl.getParameter(gl.VENDOR),
    renderer: debug ? gl.getParameter(debug.UNMASKED_RENDERER_WEBGL) : gl.getParameter(gl.RENDERER),
    version: gl.getParameter(gl.VERSION) }
  result.points = positions!.length / 3
  result.frameId = response.headers.get('X-Lingtu-Frame-Id')
  result.contentEpoch = response.headers.get('X-Lingtu-Content-Epoch')
  const scene = new THREE.Scene()
  const layer = createPcdLayer(cloud)
  scene.add(layer)
  const sourceBuffer = layer.geometry.getAttribute('position').array
  const current = layer.material
  const legacy = current.clone()
  legacy.vertexShader = legacyVertex
  legacy.fragmentShader = legacyFragment
  const camera = new THREE.PerspectiveCamera(45, 800 / 600, .01, 10000)
  const world = new THREE.Box3().setFromObject(layer)
  const center = world.getCenter(new THREE.Vector3())
  const distance = pcdCameraDistance(world.getSize(new THREE.Vector3()).length() / 2, camera.aspect)
  const pose = (angle: number) => {
    camera.position.copy(center).addScaledVector(new THREE.Vector3(Math.cos(angle), 1.5, Math.sin(angle)).normalize(), distance)
    camera.lookAt(center)
    camera.updateMatrixWorld()
  }
  const prepare = (material: THREE.ShaderMaterial, opaque = false) => {
    layer.material = material
    material.uniforms.pointSize.value = 2
    material.uniforms.opacity.value = opaque ? 1 : .9
    material.opacity = opaque ? 1 : .9
    material.transparent = !opaque
    material.depthWrite = opaque
    material.needsUpdate = true
  }
  const variants = [
    { name: 'full_fragment_color', material: legacy, opaque: false },
    { name: 'full_current_color', material: current, opaque: false },
    { name: 'full_opaque_diagnostic', material: current, opaque: true },
  ]
  result.currentColorStage = current.vertexShader.includes('cloudHeightColor') ? 'vertex' : 'fragment'
  const stats = variants.map(variant => ({ name: variant.name, cpuReadbackMs: [] as number[], gpuMs: [] as number[],
    renderedPoints: [] as number[], drawCalls: [] as number[] }))
  const pixel = new Uint8Array(4)
  const check = () => {
    assert(!gl.isContextLost(), 'WebGL context lost')
    assert(!shaderErrors.length, `Shader compilation failed: ${shaderErrors.join('\n')}`)
    const error = gl.getError()
    assert(error === gl.NO_ERROR, `WebGL error ${error}`)
    assert(renderer.info.render.points === positions!.length / 3 && renderer.info.render.calls === 1,
      `Incomplete draw: ${renderer.info.render.points} points, ${renderer.info.render.calls} calls`)
    assert(layer.geometry.index === null && layer.geometry.getAttribute('position').array === sourceBuffer,
      'The benchmark changed the complete source geometry')
  }
  const readPixel = () => gl.readPixels(400, 300, 1, 1, gl.RGBA, gl.UNSIGNED_BYTE, pixel)
  for (const variant of variants) {
    prepare(variant.material, variant.opaque)
    for (let i = 0; i < 3; i++) { pose(i * .1); renderer.render(scene, camera); readPixel(); check() }
  }
  for (let i = 0; i < 12; i++) {
    pose(i * Math.PI / 6)
    for (let order = 0; order < variants.length; order++) {
      const index = (order + i) % variants.length
      const variant = variants[index], sample = stats[index]
      status.textContent = `正在测量 ${variant.name}，视角 ${i + 1}/12，${result.points} 点…`
      prepare(variant.material, variant.opaque)
      const query = timer ? gl2.createQuery() : null
      if (timer && query) gl2.beginQuery(timer.TIME_ELAPSED_EXT, query)
      const start = performance.now()
      renderer.render(scene, camera)
      if (timer && query) gl2.endQuery(timer.TIME_ELAPSED_EXT)
      readPixel()
      sample.cpuReadbackMs.push(performance.now() - start)
      check()
      sample.renderedPoints.push(renderer.info.render.points)
      sample.drawCalls.push(renderer.info.render.calls)
      if (timer && query) {
        const deadline = performance.now() + 20000
        while (!gl2.getQueryParameter(query, gl2.QUERY_RESULT_AVAILABLE)) {
          assert(!gl.isContextLost() && performance.now() < deadline, 'GPU timer unavailable or context lost')
          await new Promise(resolve => setTimeout(resolve, 10))
        }
        assert(!gl2.getParameter(timer.GPU_DISJOINT_EXT), 'GPU timer disjoint; measurements invalid')
        sample.gpuMs.push(gl2.getQueryParameter(query, gl2.QUERY_RESULT) / 1e6)
        gl2.deleteQuery(query)
      }
      await new Promise(resolve => setTimeout(resolve, 0))
    }
  }
  const summarize = (values: number[]) => {
    if (!values.length) return null
    const sorted = [...values].sort((a, b) => a - b)
    return { medianMs: (sorted[5] + sorted[6]) / 2, p95Ms: sorted[Math.ceil(sorted.length * .95) - 1] }
  }
  result.samples = stats.map(sample => ({ ...sample, cpuReadback: summarize(sample.cpuReadbackMs), gpu: summarize(sample.gpuMs) }))
  const target = new THREE.WebGLRenderTarget(800, 600)
  const image = (material: THREE.ShaderMaterial) => {
    prepare(material)
    pose(.7)
    renderer.setRenderTarget(target)
    renderer.render(scene, camera)
    const bytes = new Uint8Array(800 * 600 * 4)
    renderer.readRenderTargetPixels(target, 0, 0, 800, 600, bytes)
    check()
    return bytes
  }
  const before = image(legacy), after = image(current)
  let maxChannelDifference = 0, changedPixels = 0, beforeContentPixels = 0, afterContentPixels = 0
  for (let i = 0; i < before.length; i += 4) {
    if (before[i + 3] && (before[i] !== 255 || before[i + 1] !== 255 || before[i + 2] !== 255)) beforeContentPixels++
    if (after[i + 3] && (after[i] !== 255 || after[i + 1] !== 255 || after[i + 2] !== 255)) afterContentPixels++
    let difference = 0
    for (let channel = 0; channel < 4; channel++) difference = Math.max(difference, Math.abs(before[i + channel] - after[i + channel]))
    maxChannelDifference = Math.max(maxChannelDifference, difference)
    if (difference) changedPixels++
  }
  result.imageComparison = { changedPixels, maxChannelDifference, beforeContentPixels, afterContentPixels, pixels: 800 * 600 }
  assert(beforeContentPixels > 100 && afterContentPixels > 100, 'Empty or nearly empty rendered point cloud')
  renderer.setRenderTarget(null)
  prepare(current)
  renderer.render(scene, camera)
  target.dispose()
  legacy.dispose()
  result.complete = true
}

try { await run() } catch (error) { result.error = error instanceof Error ? error.stack : String(error) }
status.textContent = JSON.stringify(result, null, 2)
try {
  const saved = await fetch('/__bench-result', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(result) })
  assert(saved.ok, `Result collector HTTP ${saved.status}`)
} catch (error) { status.textContent += `\nResult collection failed: ${String(error)}` }
