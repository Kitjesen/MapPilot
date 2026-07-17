import * as THREE from 'three'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js'

import { THUNDER_V4_DISPLAY_SCALE } from './thunderV4ModelSpec.ts'

type Vec3 = readonly [number, number, number]

interface LegSpec {
  prefix: 'fr' | 'fl' | 'rr' | 'rl'
  hipFile: string
  hipPosition: Vec3
  hipAxis: Vec3
  hipAngle: number
  thighPosition: Vec3
  thighAxis: Vec3
  thighAngle: number
  calfPosition: Vec3
  calfAxis: Vec3
  calfAngle: number
  footPosition: Vec3
}

const LEGS: readonly LegSpec[] = [
  { prefix: 'fr', hipFile: 'fr_hip_link.STL', hipPosition: [0.249499, -0.0649956, 0.08105], hipAxis: [-1, 0, 0], hipAngle: -0.1, thighPosition: [0.078, -0.0795, 0], thighAxis: [0, 1, 0], thighAngle: -1.2, calfPosition: [-0.249452, -0.0695, -0.01655], calfAxis: [0, 1, 0], calfAngle: 2.7, footPosition: [0.240758, -0.0729, 0.0305] },
  { prefix: 'fl', hipFile: 'fl_hip_Link.STL', hipPosition: [0.249499, 0.0650044, 0.08105], hipAxis: [-1, 0, 0], hipAngle: 0.1, thighPosition: [0.078, 0.0795, 0], thighAxis: [0, -1, 0], thighAngle: 1.2, calfPosition: [-0.249478, 0.0695, -0.01615], calfAxis: [0, -1, 0], calfAngle: -2.7, footPosition: [0.240809, 0.0729, 0.0301] },
  { prefix: 'rr', hipFile: 'rr_hip_Link.STL', hipPosition: [-0.249501, -0.0649956, 0.08105], hipAxis: [1, 0, 0], hipAngle: 0.1, thighPosition: [-0.078, -0.0795, 0], thighAxis: [0, 1, 0], thighAngle: 1.2, calfPosition: [0.249452, -0.0695, -0.01655], calfAxis: [0, 1, 0], calfAngle: -2.7, footPosition: [-0.240758, -0.0729, 0.0305] },
  { prefix: 'rl', hipFile: 'rl_hip_Link.STL', hipPosition: [-0.249501, 0.0650044, 0.08105], hipAxis: [1, 0, 0], hipAngle: -0.1, thighPosition: [-0.078, 0.0795, 0], thighAxis: [0, -1, 0], thighAngle: -1.2, calfPosition: [0.249452, 0.0695, -0.01655], calfAxis: [0, -1, 0], calfAngle: 2.7, footPosition: [-0.240758, 0.0729, 0.0305] },
]

const SENSOR_LINKS = [
  { file: 'lidar1_Link.STL', position: [0.4028760749, 0, 0.0582019451] as Vec3, quaternion: [1.492568566e-15, -0.9238795325, -6.182421429e-16, -0.3826834324] as const },
  { file: 'camera1_Link.STL', position: [0.4233588004, -0.000496203, 0.1137071496] as Vec3, quaternion: [0.405579788, -0.5792279659, 0.5792279648, -0.4055797873] as const },
  { file: 'lidar2_Link.STL', position: [-0.3063799256, 0, 0.1941716889] as Vec3, quaternion: [1.615544574e-15, 0, 0, -1] as const },
  { file: 'camera2_Link.STL', position: [-0.4259093184, 0, 0.0790135977] as Vec3, quaternion: [0.7070918521, -0.00459746036, -0.7070918352, -0.004594863066] as const },
] as const

function joint(parent: THREE.Object3D, position: Vec3, axis?: Vec3, angle = 0): THREE.Group {
  const group = new THREE.Group()
  group.position.set(...position)
  if (axis && angle !== 0) group.rotateOnAxis(new THREE.Vector3(...axis).normalize(), angle)
  parent.add(group)
  return group
}

function loadMesh(
  loader: STLLoader,
  parent: THREE.Object3D,
  filename: string,
  material: THREE.Material,
  onLoaded?: () => void,
): void {
  loader.load(
    `/robot/meshes/${encodeURIComponent(filename)}`,
    geometry => {
      geometry.computeVertexNormals()
      const mesh = new THREE.Mesh(geometry, material)
      mesh.name = `thunder-v4:${filename}`
      parent.add(mesh)
      onLoaded?.()
    },
    undefined,
    error => console.warn(`Thunder V4 mesh unavailable: ${filename}`, error),
  )
}

/** Build the nominal-standing Thunder V4 URDF link tree at a smaller UI scale. */
export function createThunderV4Model(): THREE.Group {
  const displayRoot = new THREE.Group()
  displayRoot.name = 'thunder-v4-urdf-model'
  displayRoot.scale.setScalar(THUNDER_V4_DISPLAY_SCALE)

  // URDF/STL is X-forward, Y-left, Z-up.  Scene3D is X-forward, Y-up, Z-right.
  const urdfRoot = new THREE.Group()
  urdfRoot.rotation.x = -Math.PI / 2
  displayRoot.add(urdfRoot)

  // Nominal joint pose has an 8.8 cm wheel-bottom offset at root z=0.60.
  // Re-anchor that envelope to the viewer ground plane.
  const base = joint(urdfRoot, [0, 0, 0.512])
  const loader = new STLLoader()
  const bodyMaterial = new THREE.MeshStandardMaterial({ color: 0xdbe7f5, roughness: 0.58, metalness: 0.18 })
  const wheelMaterial = new THREE.MeshStandardMaterial({ color: 0x202735, roughness: 0.9 })
  const sensorMaterial = new THREE.MeshStandardMaterial({ color: 0x38bdf8, emissive: 0x082f49, roughness: 0.5 })

  const fallback = new THREE.LineSegments(
    new THREE.EdgesGeometry(new THREE.BoxGeometry(0.86, 0.17, 0.27)),
    new THREE.LineBasicMaterial({ color: 0xf472b6 }),
  )
  fallback.rotation.x = Math.PI / 2
  fallback.position.z = 0.08
  base.add(fallback)
  loadMesh(loader, base, 'base_link.STL', bodyMaterial, () => {
    fallback.visible = false
  })

  for (const leg of LEGS) {
    const hip = joint(base, leg.hipPosition, leg.hipAxis, leg.hipAngle)
    loadMesh(loader, hip, leg.hipFile, bodyMaterial)
    const thigh = joint(hip, leg.thighPosition, leg.thighAxis, leg.thighAngle)
    loadMesh(loader, thigh, `${leg.prefix}_thigh_Link.STL`, bodyMaterial)
    const calf = joint(thigh, leg.calfPosition, leg.calfAxis, leg.calfAngle)
    loadMesh(loader, calf, `${leg.prefix}_calf_Link.STL`, bodyMaterial)
    const foot = joint(calf, leg.footPosition)
    loadMesh(loader, foot, `${leg.prefix}_foot_Link.STL`, wheelMaterial)
  }

  for (const sensor of SENSOR_LINKS) {
    const link = joint(base, sensor.position)
    const [w, x, y, z] = sensor.quaternion
    link.quaternion.set(x, y, z, w)
    loadMesh(loader, link, sensor.file, sensorMaterial)
  }

  const heading = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(0.22, 0.68, 0),
    0.42,
    0xffffff,
    0.12,
    0.07,
  )
  displayRoot.add(heading)
  return displayRoot
}
