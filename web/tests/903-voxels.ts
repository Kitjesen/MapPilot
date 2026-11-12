import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { createOctomapLayer } from '../src/components/scene3d/layers/octomapLayer'
import { parseOctomap } from '../src/services/octomap'

const element = <T extends HTMLElement>(id: string) => document.getElementById(id) as T
const view = element<HTMLDivElement>('view')
const profiles = {
  clean: { id: '903room_v6_radius_20261110', count: 1013007,
    text: 'DUFO 清理后，再移除 23,769 个邻域稀疏点。\n同一组全局规划：56/120，成功请求没有增加。' },
  dufo: { id: '903room_v6_dufo_20261110', count: 1033881,
    text: 'DUFOMap 清理并重建射线占据。\n同一组全局规划：56/120。' },
}
const renderer = new THREE.WebGLRenderer({ antialias: true })
renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5))
renderer.setClearColor(0xf7f9fb)
view.append(renderer.domElement)
const scene = new THREE.Scene()
const camera = new THREE.PerspectiveCamera(45, 1, .02, 500)
camera.up.set(0, 0, 1)
const controls = new OrbitControls(camera, renderer.domElement)
controls.enableDamping = true
let cells: Float32Array = new Float32Array()
let mesh: THREE.InstancedMesh | null = null
let bounds = new THREE.Box3(), displayedBounds = new THREE.Box3(), occupiedCount = 0

function fit(top: boolean) {
  const box = displayedBounds.isEmpty() ? bounds : displayedBounds
  if (box.isEmpty()) return
  const center = box.getCenter(new THREE.Vector3()), size = box.getSize(new THREE.Vector3())
  const distance = Math.max(size.y, size.x / camera.aspect, size.z) / (2 * Math.tan(Math.PI / 8)) * 1.3
  controls.target.copy(center)
  camera.position.copy(center).add(top ? new THREE.Vector3(0, -.001, distance) : new THREE.Vector3(.2, -.7, .8).normalize().multiplyScalar(distance))
  controls.update()
}

function show() {
  if (!cells.length) return
  const low = Number(element<HTMLInputElement>('low').value), high = Number(element<HTMLInputElement>('high').value)
  if (!Number.isFinite(low) || !Number.isFinite(high) || low >= high) return
  let count = 0
  for (let i = 0; i < cells.length; i += 4) if (cells[i + 2] >= low && cells[i + 2] <= high) count++
  const visible = new Float32Array(count * 4)
  displayedBounds = new THREE.Box3()
  let cursor = 0
  for (let i = 0; i < cells.length; i += 4) {
    if (cells[i + 2] < low || cells[i + 2] > high) continue
    visible.set(cells.subarray(i, i + 4), cursor); cursor += 4
    displayedBounds.expandByPoint(new THREE.Vector3(cells[i], cells[i + 1], cells[i + 2]))
  }
  if (mesh) { scene.remove(mesh); mesh.geometry.dispose(); (mesh.material as THREE.Material).dispose(); mesh.dispose() }
  mesh = createOctomapLayer({ resolution: .05, cells: visible,
    bounds: { min: [bounds.min.x, bounds.min.y, bounds.min.z], max: [bounds.max.x, bounds.max.y, bounds.max.z] } },
    0, 0, 'height', 1, Infinity)
  if (mesh) {
    // The shared scene layer is Y-up; this inspection page uses the map's Z-up frame.
    mesh.rotation.x = Math.PI / 2
    scene.add(mesh)
  }
  element('loaded').textContent = `占据量 ${occupiedCount.toLocaleString()} 个 5 cm 格 · 当前显示 ${count.toLocaleString()} 个占据叶 · ${low} 至 ${high} m`
}

async function load() {
  const select = element<HTMLSelectElement>('source')
  select.disabled = true
  const key = select.value as keyof typeof profiles, profile = profiles[key]
  element('map-name').textContent = profile.id
  element('source-path').textContent = `artifacts/maps/903-clean-20261110/maps/${profile.id}/octomap.ot`
  element('status').textContent = profile.text
  element('loaded').textContent = '正在读取实际占据格…'
  try {
    const response = await fetch(`/@fs/D:/inovxio/brain/lingtu/artifacts/maps/903-clean-20261110/maps/${profile.id}/octomap.ot`)
    if (!response.ok) throw new Error(`读取失败：HTTP ${response.status}`)
    const map = parseOctomap(await response.arrayBuffer())
    occupiedCount = 0
    for (let i = 3; i < map.cells.length; i += 4) occupiedCount += Math.round((map.cells[i] / map.resolution) ** 3)
    if (map.resolution !== .05 || occupiedCount !== profile.count) throw new Error('占据数量或分辨率与原生记录不一致')
    cells = map.cells
    bounds = new THREE.Box3(new THREE.Vector3(...map.bounds.min), new THREE.Vector3(...map.bounds.max))
    show(); fit(false)
  } catch (error) { element('loaded').textContent = String(error) }
  finally { select.disabled = false }
}

element('source').onchange = () => { void load() }
element('low').onchange = show; element('high').onchange = show
element('top').onclick = () => fit(true)
element('perspective').onclick = () => fit(false)
element('detail').onclick = () => { controls.target.set(-11.8, 9.1, 0); camera.position.set(-10, 5.7, 2); controls.update() }
element('floor').onclick = () => { element<HTMLInputElement>('low').value = '-0.8'; element<HTMLInputElement>('high').value = '0.8'; show() }
element('full').onclick = () => {
  element<HTMLInputElement>('low').value = String(Math.floor(bounds.min.z * 10) / 10)
  element<HTMLInputElement>('high').value = String(Math.ceil(bounds.max.z * 10) / 10)
  show(); fit(false)
}
new ResizeObserver(() => {
  renderer.setSize(view.clientWidth, view.clientHeight)
  camera.aspect = view.clientWidth / view.clientHeight; camera.updateProjectionMatrix()
}).observe(view)
renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera) })
void load()
