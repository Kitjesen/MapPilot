import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { createPcdLayer } from '../src/components/scene3d/layers/pcdLayer'
import { parsePcd } from '../src/services/pcdPreview'
import originalEvidence from '../../artifacts/nav/903-inspect-20261110/data.json'
import dufoEvidence from '../../artifacts/maps/903-clean-20261110/viewer-data.json'

const dufo = new URLSearchParams(location.search).get('map') === 'dufo'
const evidence = dufo ? dufoEvidence : originalEvidence

const element = <T extends HTMLElement>(id: string) => document.getElementById(id) as T
const view = element<HTMLDivElement>('view')
const candidateName = dufo ? 'DUFO 清理候选' : '局部候选'
const summaryText = dufo
  ? '完整 120 请求：原图 111 成功，新候选 56 成功。\n六组跨区域请求：原图 3/12，新候选 5/12。\n新候选未通过导航回归，未设为活动地图。'
  : 'A→B 和 B→A 是两次请求。\n六组共 12 次，原图与局部候选均成功 3 次、失败 9 次。\n这些位置尚未全部确认现场可连通，不能把九次失败都当成算法错误。'
element('outcome').textContent = summaryText
if (dufo) {
  document.title = '903 DUFOMap 清理候选'
  element('title').textContent = '清理已完成 · 尚未验收'
  element('map-name').textContent = evidence.map
  element('description').textContent = '用 Go2 取回的 605 帧扫描重新清理：4,452,709 → 4,173,772 点。沿用既有回环优化，重新生成 5 cm 射线地图。导航回归由 111/120 降至 56/120，保留为候选，不替换原图。'
  element('description').classList.add('warn')
  element<HTMLAnchorElement>('alternate').href = '/tests/903-inspect.html'
  element('alternate').textContent = '返回原保存图对照'
  element('cloud-label').textContent = '新清理点云（完整读取，无抽样）'
  element('old-label').textContent = '原图路线参考（新图未通过）'
  element('candidate-label').textContent = '新图该长路线未生成'
  element('source-description').textContent = '2026-11-10 从 Go2 取回完整 v6 档案，在本机副本使用当前 DUFOMap 清理并重放射线；未更换机器人活动地图。'
  element('source-files').textContent = 'artifacts/maps/903-clean-20261110/maps/903room_v6_dufo_20261110/ — map.pcd、octomap.ot、metadata.json、.nav/'
  element<HTMLInputElement>('old').checked = false
  element<HTMLInputElement>('candidate').disabled = true
}
const renderer = new THREE.WebGLRenderer({ antialias: true })
renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5))
renderer.setClearColor(0xfafafa)
renderer.localClippingEnabled = true
view.append(renderer.domElement)
const scene = new THREE.Scene()
const camera = new THREE.PerspectiveCamera(45, 1, .02, 1000)
camera.up.set(0, 0, 1)
const controls = new OrbitControls(camera, renderer.domElement)
controls.enableDamping = true
const grid = new THREE.GridHelper(80, 80, 0xd3d9dd, 0xe6e9eb)
grid.rotation.x = Math.PI / 2
grid.position.set(-9, 2, -.6)
scene.add(grid)
const top = (x = -11, y = 0, distance = 46) => {
  controls.target.set(x, y, -.3)
  camera.position.set(x, y - .01, distance)
  controls.update()
}
top()
element('top').onclick = () => top()
element('perspective').onclick = () => {
  controls.target.set(-9, 2, 0)
  camera.position.set(20, -28, 32)
  controls.update()
}
element('repair').onclick = () => top(-19.5, 5.5, 19)
const line = (points: number[][], color: number, zOffset = 0) => {
  const geometry = new THREE.BufferGeometry().setFromPoints(points.map(p => new THREE.Vector3(p[0], p[1], p[2] + zOffset)))
  const result = new THREE.Line(geometry, new THREE.LineBasicMaterial({ color, depthTest: false, depthWrite: false }))
  result.renderOrder = 10
  return result
}
const oldRoute = line(evidence.originalRoute, 0xc9484e, .05)
oldRoute.visible = !dufo
const candidate = line(evidence.candidateRoute, 0x21866b, .06)
candidate.visible = false
scene.add(oldRoute, candidate)
const repairBox = line([[-26, 3, .15], [-13, 3, .15], [-13, 8, .15], [-26, 8, .15], [-26, 3, .15]], 0xb17b19)
scene.add(repairBox)
const label = (text: string, p: number[], color: string) => {
  const canvas = document.createElement('canvas')
  canvas.width = 160; canvas.height = 72
  const ctx = canvas.getContext('2d')!
  ctx.fillStyle = '#fffffff0'; ctx.fillRect(0, 0, 160, 72)
  ctx.font = 'bold 34px sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'
  ctx.fillStyle = color; ctx.fillText(text, 80, 36)
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(canvas), depthTest: false }))
  sprite.position.set(p[0], p[1], p[2] + .5)
  sprite.scale.set(1.3, .58, 1)
  sprite.renderOrder = 20
  return sprite
}
scene.add(label('走廊 A', [-18, 8, .15], '#94631a'))
const markers = new THREE.Group()
const selection = new THREE.Group()
scene.add(markers, selection)
for (const pair of evidence.pairs) {
  const option = document.createElement('option')
  option.value = String(pair.number)
  option.textContent = `第 ${pair.number} 组：${(dufo ? pair.candidate : pair.original).filter(r => r.ok).length}/2 成功`
  element<HTMLSelectElement>('pair').append(option)
  markers.add(label(`${pair.number}A`, pair.source, '#8d3265'), label(`${pair.number}B`, pair.goal, '#175e9e'))
}
const reason = (r: { ok: boolean; error?: string }) => r.ok ? '生成路线' : r.error === 'global_planning_timeout' ? '超时，未生成路线' : '未找到路线'
element<HTMLSelectElement>('pair').onchange = event => {
  while (selection.children.length) {
    const object = selection.children[0] as THREE.Line
    selection.remove(object); object.geometry.dispose(); (object.material as THREE.Material).dispose()
  }
  const pair = evidence.pairs.find(p => String(p.number) === (event.target as HTMLSelectElement).value)
  if (!pair) {
    element('outcome').textContent = summaryText
    return
  }
  const format = (point: number[]) => point.map(x => x.toFixed(3)).join(', ')
  element('outcome').textContent = `第 ${pair.number} 组\nA = (${format(pair.source)})\nB = (${format(pair.goal)})\n原图 A→B：${reason(pair.original[0])}\n原图 B→A：${reason(pair.original[1])}\n${candidateName} A→B：${reason(pair.candidate[0])}\n${candidateName} B→A：${reason(pair.candidate[1])}\n图上灰线仅连接两个测试点，不是规划结果。`
  selection.add(line([pair.source, pair.goal], 0x888888, .1))
  for (const result of dufo ? pair.candidate : pair.original) if (result.ok && 'path' in result) selection.add(line(result.path, 0x21866b, .15))
  const middle = pair.source.map((x, i) => (x + pair.goal[i]) / 2)
  top(middle[0], middle[1], 12)
}
element<HTMLInputElement>('old').onchange = event => { oldRoute.visible = (event.target as HTMLInputElement).checked }
element<HTMLInputElement>('candidate').onchange = event => { candidate.visible = (event.target as HTMLInputElement).checked }
element<HTMLInputElement>('markers').onchange = event => { markers.visible = (event.target as HTMLInputElement).checked }

async function load() {
  const response = await fetch(dufo
    ? '/@fs/D:/inovxio/brain/lingtu/artifacts/maps/903-clean-20261110/maps/903room_v6_dufo_20261110/map.pcd'
    : '/@fs/D:/inovxio/brain/lingtu/build/nav-field-20261025/active-v6-map.pcd')
  if (!response.ok) throw new Error(`点云读取失败：HTTP ${response.status}`)
  const positions = parsePcd(await response.arrayBuffer())
  if (!positions || positions.length / 3 !== evidence.sourcePoints) throw new Error('点云数量与留存证据不一致')
  const bounds = new THREE.Box3().setFromBufferAttribute(new THREE.BufferAttribute(positions, 3))
  const cloud = createPcdLayer({ positions, bounds: { min: bounds.min.toArray(), max: bounds.max.toArray() }, colorRange: { min: -.5, max: .8 }, sourceCount: evidence.sourcePoints })
  // The inspection canvas uses the source map's Z-up coordinates throughout.
  cloud.matrix.identity()
  const clip = () => {
    const low = Number(element<HTMLInputElement>('low').value), high = Number(element<HTMLInputElement>('high').value)
    if (!Number.isFinite(low) || !Number.isFinite(high) || low >= high) return
    cloud.material.clippingPlanes = [new THREE.Plane(new THREE.Vector3(0, 0, 1), -low), new THREE.Plane(new THREE.Vector3(0, 0, -1), high)]
    cloud.material.uniforms.heightRange.value.set(low, high)
    cloud.material.needsUpdate = true
  }
  element('low').onchange = clip; element('high').onchange = clip
  const height = (low: number, high: number) => {
    element<HTMLInputElement>('low').value = String(low); element<HTMLInputElement>('high').value = String(high); clip()
  }
  element('floor').onclick = () => height(-.8, .8)
  element('full').onclick = () => height(Math.floor(bounds.min.z * 10) / 10, Math.ceil(bounds.max.z * 10) / 10)
  element<HTMLInputElement>('cloud').onchange = event => { cloud.visible = (event.target as HTMLInputElement).checked }
  clip(); scene.add(cloud)
  const raycaster = new THREE.Raycaster()
  raycaster.params.Points.threshold = .05
  renderer.domElement.ondblclick = event => {
    const rect = renderer.domElement.getBoundingClientRect()
    raycaster.setFromCamera(new THREE.Vector2((event.clientX - rect.left) / rect.width * 2 - 1, 1 - (event.clientY - rect.top) / rect.height * 2), camera)
    const low = Number(element<HTMLInputElement>('low').value), high = Number(element<HTMLInputElement>('high').value)
    const hit = raycaster.intersectObject(cloud).find(hit => hit.point.z >= low && hit.point.z <= high)
    if (hit) element('pick').textContent = `选中原始点：X ${hit.point.x.toFixed(3)} m，Y ${hit.point.y.toFixed(3)} m，Z ${hit.point.z.toFixed(3)} m（查看坐标，不是下发目标）`
  }
  const graphData = await fetch(dufo
    ? '/@fs/D:/inovxio/brain/lingtu/artifacts/maps/903-clean-20261110/graph.json'
    : '/@fs/D:/inovxio/brain/lingtu/artifacts/nav/903-inspect-20261110/graph.json').then(r => r.json())
  const graph = new THREE.Group()
  for (const points of graphData) graph.add(line(points, 0x3d86bc, .01))
  graph.visible = false; scene.add(graph)
  element<HTMLInputElement>('graph').onchange = event => { graph.visible = (event.target as HTMLInputElement).checked }
  element('loaded').textContent = `已载入 ${evidence.sourcePoints.toLocaleString()} 个${dufo ? '清理保留点' : '原始点'} · 地图 5 cm · 黄色框为上轮走廊 A`
}
const resize = () => {
  renderer.setSize(view.clientWidth, view.clientHeight)
  camera.aspect = view.clientWidth / view.clientHeight; camera.updateProjectionMatrix()
}
new ResizeObserver(resize).observe(view)
renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera) })
void load().catch(error => { element('loaded').textContent = String(error) })
