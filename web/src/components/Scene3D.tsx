import { scanCanStandAlone } from '../services/sceneTelemetry.ts'
import type { ReadyPlanningMap } from '../services/planningMap.ts'
import { createPlanningMapLayer } from './scene3d/layers/planningMapLayer.ts'
import { mapProjectionDisplayZ, riskProjectionDisplayZ } from '../services/mapProjectionHeight.ts'
import type { MappingObservationState } from '../services/mappingObservation.ts'
import { createMappingObservationLayer } from './scene3d/layers/mappingObservationLayer.ts'
/**
 * Scene3D — Three.js 3D map visualization
 *
 * Coordinate mapping:
 *   World (LingTu): X forward, Y left, Z up
 *   Three.js:       X right, Y up,      Z toward camera
 *   → Three.js pos = (worldX, worldZ_height, -worldY)
 */
import { useRef, useEffect, forwardRef, useImperativeHandle } from 'react'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import type { PathPoint, SceneGraphEvent, NavigationDdsSnapshotResponse } from '../types'
import type { BinaryCloud } from '../hooks/useBinaryCloud'
import {
  cloudFrameSharesSavedMapFrame,
  cloudFramesShareCoordinateEpoch,
} from '../workers/cloudDecoderCore.ts'
import { createNativeTraversabilityLayer, type NativeTraversabilityLayerState } from './scene3d/layers/traversabilityLayer.ts'
import { createElevationLayer, type ElevationLayerState } from './scene3d/layers/elevationLayer.ts'
import { disposeLiveCloudLayer, upsertLiveCloudLayer } from './scene3d/layers/liveCloudLayer'
import {
  createSavedMapLayer,
  SAVED_MAP_Z_CEIL,
  SAVED_MAP_Z_FLOOR,
  updateSavedMapPointSize,
} from './scene3d/layers/savedMapLayer'
import { disposeGroupedMesh, type GroupedMesh } from './scene3d/layers/layerUtils'
import { lingtuToThree, threeToLingtu, lingtuQuaternionToThree } from '../services/coordinateFrame.ts'
import { createThunderV4Model } from './scene3d/robot/thunderV4Model'
import { createGo2Model, type Go2Model } from './scene3d/robot/go2Model.ts'
import type { JointTelemetryStream } from '../services/jointTelemetryStream.ts'
import { createSafetyEnvelope, type SafetyEnvelope } from './scene3d/layers/safetyEnvelope.ts'
import { PoseInterpolation } from './scene3d/robot/poseInterpolation.ts'
import { createPathDisplay, PATH_COLORS } from './scene3d/layers/pathDisplay.ts'
import {
  createLocalPlannerDiagnosticLayer,
  disposeLocalPlannerDiagnosticLayer,
} from './scene3d/layers/localPlannerLayer'

export interface Scene3DHandle {
  resetCamera(): void
  topView(height?: number): void
  fitMap(): void
}

interface Layers {
  grid:    boolean
  cloud:   boolean
  trail:   boolean
  path:    boolean
  goal:    boolean
  robot:   boolean
  elevation: boolean
  nativeTraversability: boolean
  localPlanner: boolean
}

interface Scene3DProps {
  cloud:        BinaryCloud
  scanCloud?:   BinaryCloud | null
  savedMapFlat?: number[]
  savedMapFrameId?: string | null
  savedMapEpoch?: number | null
  savedMapVisible?: boolean
  planningMap?: ReadyPlanningMap | null
  planningMapVisible?: boolean
  mappingMode?: boolean
  mappingObservation?: MappingObservationState
  mappingObservationVisible?: boolean
  scanVisible?: boolean
  elevationState: ElevationLayerState
  nativeTraversabilityState: NativeTraversabilityLayerState
  sceneGraph:   SceneGraphEvent | null
  robotX:       number
  robotY:       number
  robotZ?:      number
  orientation?: [number, number, number, number] | null
  poseStampS?:  number | null
  poseEpoch?:   string | number | null
  robotModel?:  'go2' | 'thunder_v4'
  jointTelemetry?: JointTelemetryStream
  followRobot?: boolean
  robotValid:   boolean
  yaw:          number
  trail:        Array<[number, number, number?]>
  path:         PathPoint[]
  localPath:    PathPoint[]
  localPlannerSnapshot?: NavigationDdsSnapshotResponse | null
  safetyEnvelope?: SafetyEnvelope | null
  safetyView?: 'slice' | 'volume'
  layers:       Layers
  pointSize:    number
  onPendingGoal?: (x: number, y: number) => void
  onRelocalize?: (x: number, y: number) => void
  pendingGoal?:  { x: number; y: number; z?: number } | null
  pendingGoalRadius?: number
  pendingGoalLabel?: string
  previewPath?: PathPoint[]
}

const LIVE_SCAN_COLOR = 0x587b95
// The worker owns binary decode, filtering, color mapping, and coordinate
// conversion. Scene3D consumes those typed arrays as a true 3D Points layer.

interface GridTransform {
  rows: number
  cols: number
  resolution: number
  origin: [number, number] | [number, number, number]
  yaw?: number
}

function gridTransformKey(grid: GridTransform): string {
  return `${grid.rows}:${grid.cols}:${grid.resolution}:${grid.origin[0]}:${grid.origin[1]}:${grid.yaw ?? 0}`
}

function elevationTransformKey(layer: GridTransform, minZ: number, maxZ: number): string {
  return `${gridTransformKey(layer)}:${minZ}:${maxZ}`
}

function removeFrom(scene: THREE.Scene, obj: THREE.Object3D | undefined | null) {
  if (!obj) return
  scene.remove(obj)
  const geometries = new Set<THREE.BufferGeometry>()
  const materials = new Set<THREE.Material>()
  obj.traverse(child => {
    const mesh = child as THREE.Mesh
    if (mesh.geometry) geometries.add(mesh.geometry)
    if (mesh.material) (Array.isArray(mesh.material) ? mesh.material : [mesh.material])
      .forEach(material => materials.add(material))
  })
  geometries.forEach(geometry => geometry.dispose())
  materials.forEach(material => material.dispose())
}

function createPoseMarker(): THREE.Group {
  const group = new THREE.Group()
  group.name = 'unidentified-robot-pose'
  group.add(new THREE.Mesh(new THREE.SphereGeometry(0.06, 16, 12),
    new THREE.MeshStandardMaterial({ color: 0x334155, roughness: 0.65 })))
  group.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(0, 0, 0), 0.4, 0x087cc4, 0.085, 0.045))
  return group
}

export const Scene3D = forwardRef<Scene3DHandle, Scene3DProps>(function Scene3D(
  { cloud, scanCloud, savedMapFlat, savedMapFrameId, savedMapEpoch, savedMapVisible = true, planningMap, planningMapVisible = true, mappingMode = false, mappingObservation, mappingObservationVisible = false, scanVisible = true, elevationState, nativeTraversabilityState, sceneGraph, robotX, robotY, robotZ = 0, orientation, poseStampS, poseEpoch, robotModel, jointTelemetry, followRobot = false, robotValid, yaw, trail, path, localPath, localPlannerSnapshot, safetyEnvelope, safetyView = 'slice', layers, pointSize, onPendingGoal, onRelocalize, pendingGoal, pendingGoalRadius = 0.25, pendingGoalLabel = '待确认目标', previewPath },
  ref,
) {
  const mountRef   = useRef<HTMLDivElement>(null)
  const sceneRef   = useRef<THREE.Scene | null>(null)
  const cameraRef  = useRef<THREE.PerspectiveCamera | null>(null)
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null)
  const controlsRef = useRef<OrbitControls | null>(null)
  const rafRef     = useRef(0)
  const scanFrameAligned = scanCloud != null
    && (cloudFramesShareCoordinateEpoch(cloud, scanCloud) || scanCanStandAlone(scanCloud, cloud))
  const savedMapFrameAligned = cloudFrameSharesSavedMapFrame(
    cloud,
    savedMapFrameId,
    savedMapEpoch,
  )
  const hasSavedMap = savedMapFlat !== undefined && savedMapFlat.length >= 3
  const liveCloudFrameAllowed = !hasSavedMap || savedMapFrameAligned

  // Scene objects — recreated on data change
  const trailLineRef = useRef<THREE.Line | null>(null)
  const pathLineRef  = useRef<THREE.Group | null>(null)
  const localPathRef = useRef<THREE.Group | null>(null)
  const robotRef   = useRef<THREE.Group | null>(null)
  const go2Ref = useRef<Go2Model | null>(null)
  const jointTelemetryRef = useRef(jointTelemetry)
  const goalRef        = useRef<THREE.Mesh | null>(null)
  const elevationMeshRef = useRef<GroupedMesh | null>(null)
  const mappingObservationRef = useRef<GroupedMesh | null>(null)
  const planningMapMeshRef = useRef<GroupedMesh | null>(null)
  const nativeTraversabilityMeshRef = useRef<GroupedMesh | null>(null)
  const localPlannerRef = useRef<THREE.Group | null>(null)
  const gridRef        = useRef<THREE.GridHelper | null>(null)
  const floorRef   = useRef<THREE.Mesh | null>(null)
  const liveCloudRef   = useRef<THREE.Points | null>(null)
  const scanCloudRef   = useRef<THREE.Points | null>(null)
  const savedMapRef    = useRef<THREE.Points | null>(null)
  const pointSizeRef   = useRef(pointSize)
  const sgGroupRef     = useRef<THREE.Group | null>(null)
  const raycaster  = useRef(new THREE.Raycaster())
  const robotPosRef = useRef({ x: 0, y: 0, z: 0 })
  const poseInterpolationRef = useRef(new PoseInterpolation())
  const followRef = useRef(followRobot)
  const followAnchorRef = useRef<THREE.Vector3 | null>(null)
  const robotVisibleRef = useRef(false)
  const robotLoadStatusRef = useRef<HTMLDivElement>(null)
  const observationDisplayZ = mappingObservation?.status === 'ready'
    ? mapProjectionDisplayZ(mappingObservation.layer.origin[2], robotZ, robotModel, robotValid) : undefined
  const planningDisplayZ = planningMap
    ? mapProjectionDisplayZ(planningMap.origin[2], robotZ, robotModel, robotValid) : undefined
  const underlayDisplayZ = mappingObservationVisible ? observationDisplayZ
    : planningMapVisible ? planningDisplayZ : undefined
  const riskDisplayZ = nativeTraversabilityState.status === 'ready'
    ? riskProjectionDisplayZ(nativeTraversabilityState.event.origin[2], robotZ, robotModel, robotValid, underlayDisplayZ)
    : undefined

  // ── Expose resetCamera ──────────────────────────────────────────
  useImperativeHandle(ref, () => ({
    fitMap() {
      const camera = cameraRef.current
      const controls = controlsRef.current
      const raster = planningMapMeshRef.current ?? mappingObservationRef.current
      const points = raster ? raster._group ?? raster
        : savedMapRef.current?.visible ? savedMapRef.current : liveCloudRef.current
      if (!camera || !controls || !points) return
      const bounds = new THREE.Box3().setFromObject(points)
      if (bounds.isEmpty()) return
      const center = bounds.getCenter(new THREE.Vector3())
      const size = bounds.getSize(new THREE.Vector3())
      const halfFov = THREE.MathUtils.degToRad(camera.fov / 2)
      const distance = Math.max(size.z, size.x / camera.aspect, 2) / (2 * Math.tan(halfFov)) * 1.15
      camera.position.set(center.x, bounds.max.y + distance, center.z + 0.001)
      controls.target.copy(center)
      controls.update()
    },
    topView(height = 3.5) {
      const { x, y, z } = robotPosRef.current
      const [tx, ty, tz] = lingtuToThree([x, y, z])
      cameraRef.current?.position.set(tx, ty + height, tz + 0.001)
      if (controlsRef.current) {
        controlsRef.current.target.set(tx, ty, tz)
        controlsRef.current.update()
      }
    },
    resetCamera() {
      const { x, y, z } = robotPosRef.current
      const [tx, ty, tz] = lingtuToThree([x, y, z])
      cameraRef.current?.position.set(tx - 2.8, ty + 2.3, tz + 3.4)
      if (controlsRef.current) {
        controlsRef.current.target.set(tx, ty, tz)
        controlsRef.current.update()
      }
    },
  }), [])

  // ── Init (once) ─────────────────────────────────────────────────
  useEffect(() => {
    const mount = mountRef.current!
    const w = mount.clientWidth, h = mount.clientHeight

    const scene = new THREE.Scene()
    scene.background = new THREE.Color(0xf3f3f3)
    sceneRef.current = scene

    const camera = new THREE.PerspectiveCamera(48, w / h, 0.02, 500)
    camera.position.set(-3.8, 3.2, 4.8)
    camera.lookAt(0, 0, 0)
    cameraRef.current = camera

    const renderer = new THREE.WebGLRenderer({ antialias: true })
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
    renderer.setSize(w, h)
    renderer.toneMapping = THREE.ACESFilmicToneMapping
    renderer.toneMappingExposure = 1.15
    mount.appendChild(renderer.domElement)
    rendererRef.current = renderer

    const controls = new OrbitControls(camera, renderer.domElement)
    controls.enableDamping  = true
    controls.dampingFactor  = 0.1
    controls.minDistance    = 0.75
    controls.maxDistance    = 120
    controls.maxPolarAngle  = Math.PI * 0.475
    controlsRef.current = controls

    scene.add(new THREE.HemisphereLight(0xffffff, 0x888888, 2.1))
    const dir = new THREE.DirectionalLight(0xffffff, 2.0)
    dir.position.set(4, 7, 5)
    scene.add(dir)
    const rim = new THREE.DirectionalLight(0xffffff, 1.4)
    rim.position.set(-4, 2, -5)
    scene.add(rim)

    const grid = new THREE.GridHelper(100, 100, 0xc5c5c5, 0xdedede)
    grid.material.vertexColors = false
    grid.position.y = -0.02
    scene.add(grid)
    gridRef.current = grid

    const applyTheme = () => {
      const dark = document.documentElement.dataset.theme === 'dark'
      ;(scene.background as THREE.Color).setHex(dark ? 0x191919 : 0xf3f3f3)
      grid.material.color.setHex(dark ? 0x303030 : 0xdedede)
    }
    applyTheme()
    const themeObserver = new MutationObserver(applyTheme)
    themeObserver.observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] })

    // Invisible floor plane for click raycasting
    const floor = new THREE.Mesh(
      new THREE.PlaneGeometry(600, 600),
      new THREE.MeshBasicMaterial({ visible: false, side: THREE.DoubleSide }),
    )
    floor.rotation.x = -Math.PI / 2
    scene.add(floor)
    floorRef.current = floor

    const posePosition = new THREE.Vector3()
    const followDelta = new THREE.Vector3()
    const animate = () => {
      rafRef.current = requestAnimationFrame(animate)
      const joints = jointTelemetryRef.current?.getSnapshot()
      if (joints?.connected && joints.sample && go2Ref.current) {
        const nowMs = performance.now()
        go2Ref.current.ingestJointState(joints.sample, nowMs)
        go2Ref.current.updateJointPose(nowMs)
      }
      const pose = poseInterpolationRef.current.sample(performance.now())
      const robot = robotRef.current
      if (robot) robot.visible = robotVisibleRef.current && pose !== null
      if (pose) {
        const position = posePosition.set(...lingtuToThree([pose.x, pose.y, pose.z]))
        if (robot) {
          robot.position.copy(position)
          if (pose.orientation) robot.quaternion.fromArray(lingtuQuaternionToThree(pose.orientation)).normalize()
          else robot.rotation.set(0, pose.yaw, 0)
        }
        if (followRef.current) {
          const previous = followAnchorRef.current
          const delta = followDelta.copy(position).sub(previous ?? controls.target)
          camera.position.add(delta)
          controls.target.add(delta)
          if (previous) previous.copy(position)
          else followAnchorRef.current = position.clone()
        }
      } else {
        followAnchorRef.current = null
      }
      controls.update()
      renderer.render(scene, camera)
    }
    animate()

    const ro = new ResizeObserver(() => {
      const nw = mount.clientWidth, nh = mount.clientHeight
      camera.aspect = nw / nh
      camera.updateProjectionMatrix()
      renderer.setSize(nw, nh)
    })
    ro.observe(mount)

    return () => {
      cancelAnimationFrame(rafRef.current)
      themeObserver.disconnect()
      ro.disconnect()
      controls.dispose()
      for (const child of [...scene.children]) removeFrom(scene, child)
      renderer.dispose()
      if (mount.contains(renderer.domElement)) mount.removeChild(renderer.domElement)
    }
  }, [])

  // ── Grid visibility ─────────────────────────────────────────────
  useEffect(() => {
    if (gridRef.current) gridRef.current.visible = layers.grid
  }, [layers.grid])

  // Live point cloud, Rerun-style: one 3D layer with positions and colors.
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return

    if (!layers.cloud || !liveCloudFrameAllowed) {
      if (liveCloudRef.current) {
        disposeLiveCloudLayer(scene, liveCloudRef.current)
        liveCloudRef.current = null
      }
      return
    }

    liveCloudRef.current = upsertLiveCloudLayer(scene, liveCloudRef.current, cloud, pointSize, {
      color: mappingMode ? 0x368f7c : 0xffffff,
      opacity: 0.98,
      pointSizeScale: 1.5,
      renderOrder: 6,
      vertexColors: !mappingMode,
    })
    if (liveCloudRef.current) (liveCloudRef.current.material as THREE.PointsMaterial).toneMapped = false
  }, [cloud, layers.cloud, liveCloudFrameAllowed, pointSize, mappingMode])

  // Current scan overlay.  This is intentionally separate from the accumulated
  // map cloud so mapping mode can be stable and still show live sensor motion.
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    if (!scanVisible || !liveCloudFrameAllowed || !scanCloud || !scanFrameAligned) {
      if (scanCloudRef.current) scanCloudRef.current.visible = false
      return
    }
    scanCloudRef.current = upsertLiveCloudLayer(scene, scanCloudRef.current, scanCloud, pointSize, {
      color: LIVE_SCAN_COLOR,
      opacity: 0.92,
      pointSizeScale: 1.05,
      renderOrder: 12,
      vertexColors: false,
    })
  }, [scanCloud, scanFrameAligned, scanVisible, liveCloudFrameAllowed, pointSize])

  // ── Trail ───────────────────────────────────────────────────────
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    if (trailLineRef.current) { removeFrom(scene, trailLineRef.current); trailLineRef.current = null }
    if (!layers.trail || trail.length < 2) return

    // Old XY-only history is a reference-plane projection, not invented height.
    const pts = trail.map(([x, y, z]) => new THREE.Vector3(...lingtuToThree([x, y, (z ?? 0) + 0.02])))
    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(pts),
      new THREE.LineBasicMaterial({ color: PATH_COLORS.trail, transparent: true, opacity: 0.78 }),
    )
    scene.add(line)
    trailLineRef.current = line
  }, [trail, layers.trail])

  // Paths follow the published sample segments, without a second spline fit.
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    removeFrom(scene, pathLineRef.current)
    pathLineRef.current = layers.path ? createPathDisplay(path, 'global') : null
    if (pathLineRef.current) scene.add(pathLineRef.current)
  }, [path, layers.path])

  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    removeFrom(scene, localPathRef.current)
    localPathRef.current = layers.path ? createPathDisplay(localPath, 'local') : null
    if (localPathRef.current) scene.add(localPathRef.current)
  }, [localPath, layers.path])

  // A model is selected only from the reported robot identity.
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    const go2 = robotModel === 'go2' ? createGo2Model() : null
    go2Ref.current = go2
    const robot = go2 ?? (robotModel === 'thunder_v4' ? createThunderV4Model() : createPoseMarker())
    const status = robotLoadStatusRef.current
    if (status) status.textContent = go2 ? '正在加载 Go2 官方模型…' : ''
    let disposed = false
    if (go2) void go2.ready.then(() => {
      if (disposed || !status) return
      status.textContent = go2.userData.loadState === 'error' ? 'Go2 官方模型加载失败，请刷新重试' : ''
    })
    robot.visible = false
    scene.add(robot)
    robotRef.current = robot
    return () => {
      disposed = true
      go2?.cancelLoading()
      if (status) status.textContent = ''
      removeFrom(scene, robot)
      if (robotRef.current === robot) robotRef.current = null
      if (go2Ref.current === go2) go2Ref.current = null
    }
  }, [robotModel])

  useEffect(() => {
    jointTelemetryRef.current = jointTelemetry
  }, [jointTelemetry])

  useEffect(() => {
    const z = mappingObservationVisible && observationDisplayZ !== undefined ? observationDisplayZ
      : planningMapVisible && planningDisplayZ !== undefined ? planningDisplayZ
        : mapProjectionDisplayZ(0, robotZ, robotModel, robotValid)
    if (gridRef.current) gridRef.current.position.y = z - .012
    if (floorRef.current) floorRef.current.position.y = z - .012
  }, [mappingObservationVisible, observationDisplayZ, planningMapVisible, planningDisplayZ, robotZ, robotModel, robotValid])

  useEffect(() => {
    const robot = robotRef.current
    if (!robot || !safetyEnvelope || !layers.localPlanner) return
    const envelope = createSafetyEnvelope(safetyEnvelope)
    robot.add(envelope)
    return () => {
      robot.remove(envelope)
      if (sceneRef.current) removeFrom(sceneRef.current, envelope)
    }
  }, [robotModel, safetyEnvelope, layers.localPlanner])

  useEffect(() => {
    robotVisibleRef.current = layers.robot && robotValid
    const pose = robotValid
      ? { x: robotX, y: robotY, z: robotZ, yaw, orientation, stampS: poseStampS, epoch: poseEpoch } : null
    poseInterpolationRef.current.update(pose, performance.now())
    if (robotRef.current && !robotVisibleRef.current) robotRef.current.visible = false
    if (pose) robotPosRef.current = pose
  }, [robotX, robotY, robotZ, robotValid, yaw, orientation, poseStampS, poseEpoch, layers.robot])

  useEffect(() => {
    followRef.current = followRobot
    followAnchorRef.current = null
  }, [followRobot])

  // ── Goal marker ─────────────────────────────────────────────────
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    if (goalRef.current) { removeFrom(scene, goalRef.current); goalRef.current = null }
    if (!layers.goal || path.length === 0) return

    const last = path[path.length - 1]
    const mesh = new THREE.Mesh(
      new THREE.SphereGeometry(0.12, 12, 8),
      new THREE.MeshBasicMaterial({ color: 0x436b91, wireframe: true }),
    )
    mesh.position.set(...lingtuToThree([last.x, last.y, (last.z ?? 0) + 0.08]))
    scene.add(mesh)
    goalRef.current = mesh
  }, [path, layers.goal])

  // ── Pending goal marker ────────────────────────────────────────
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    if (!pendingGoal) return
    const marker = new THREE.Group()
    marker.position.set(...lingtuToThree([pendingGoal.x, pendingGoal.y,
      underlayDisplayZ !== undefined ? underlayDisplayZ + 0.012 : (pendingGoal.z ?? 0) + 0.05]))
    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(pendingGoalRadius, 0.025, 8, 32),
      new THREE.MeshBasicMaterial({ color: 0xf5b841, depthTest: false }),
    )
    ring.rotation.x = Math.PI / 2
    ring.renderOrder = 50
    marker.add(ring)
    const label = document.createElement('canvas')
    label.width = 256; label.height = 64
    const context = label.getContext('2d')
    let texture: THREE.CanvasTexture | null = null
    if (context) {
      context.fillStyle = '#352914'; context.fillRect(0, 0, 256, 64)
      context.font = 'bold 30px sans-serif'; context.textAlign = 'center'; context.textBaseline = 'middle'
      context.fillStyle = '#ffd779'; context.fillText(pendingGoalLabel, 128, 32)
      texture = new THREE.CanvasTexture(label)
      const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, depthTest: false }))
      sprite.position.y = 0.45; sprite.scale.set(1.0, 0.25, 1); sprite.renderOrder = 51
      marker.add(sprite)
    }
    scene.add(marker)
    return () => { removeFrom(scene, marker); texture?.dispose() }
  }, [pendingGoal, pendingGoalRadius, pendingGoalLabel, underlayDisplayZ])

  useEffect(() => {
    const scene = sceneRef.current
    if (!scene || !previewPath || previewPath.length < 2) return
    const geometry = new THREE.BufferGeometry().setFromPoints(previewPath.map(point =>
      new THREE.Vector3(...lingtuToThree([point.x, point.y, (point.z ?? 0) + 0.06]))))
    const line = new THREE.Line(geometry, new THREE.LineDashedMaterial({
      color: 0xf5b841, dashSize: 0.12, gapSize: 0.08, depthTest: false,
    }))
    line.computeLineDistances(); line.renderOrder = 49
    scene.add(line)
    return () => removeFrom(scene, line)
  }, [previewPath])

  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    if (mappingObservationVisible && mappingObservation?.status === 'ready') {
      const { layer } = mappingObservation
      const transform = `${gridTransformKey(layer)}:${layer.origin[2]}:${layer.producer_boot_id}:${layer.reset_epoch}`
      if (mappingObservationRef.current?.userData.gridB64 === layer.grid_b64
        && mappingObservationRef.current.userData.transform === transform) return
      disposeGroupedMesh(scene, mappingObservationRef.current)
      const mesh = createMappingObservationLayer(mappingObservation)
      mappingObservationRef.current = mesh
      if (mesh) {
        mesh.userData.gridB64 = layer.grid_b64
        mesh.userData.transform = transform
        scene.add(mesh._group ?? mesh)
      }
    } else {
      disposeGroupedMesh(scene, mappingObservationRef.current)
      mappingObservationRef.current = null
    }
  }, [mappingObservation, mappingObservationVisible])

  useEffect(() => {
    const mesh = mappingObservationRef.current
    if (mesh && observationDisplayZ !== undefined) (mesh._group ?? mesh).position.y = observationDisplayZ
  }, [observationDisplayZ, mappingObservation, mappingObservationVisible])

  // Elevation is the map_scene minimum-observed-Z raster rendered as actual
  // displaced geometry. Invalid/stale identity states never reach the scene.
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return

    if (layers.elevation && elevationState.status === 'ready') {
      const { layer } = elevationState
      const transform = elevationTransformKey(layer, elevationState.minZ, elevationState.maxZ)
      if (elevationMeshRef.current?.userData.gridB64 === layer.grid_b64
        && elevationMeshRef.current.userData.transform === transform) return
    }
    disposeGroupedMesh(scene, elevationMeshRef.current)
    elevationMeshRef.current = null
    if (!layers.elevation || elevationState.status !== 'ready') return

    const mesh = createElevationLayer(elevationState)
    if (!mesh) return
    const { layer } = elevationState
    mesh.userData.gridB64 = layer.grid_b64
    mesh.userData.transform = elevationTransformKey(layer, elevationState.minZ, elevationState.maxZ)
    scene.add(mesh._group ?? mesh)
    elevationMeshRef.current = mesh
  }, [elevationState, layers.elevation])

  useEffect(() => {
    const scene = sceneRef.current
    if (!scene || !planningMap || !planningMapVisible) return
    const mesh = createPlanningMapLayer(planningMap)
    if (!mesh) return
    planningMapMeshRef.current = mesh
    scene.add(mesh._group ?? mesh)
    return () => { disposeGroupedMesh(scene, mesh); planningMapMeshRef.current = null }
  }, [planningMap, planningMapVisible])

  useEffect(() => {
    const mesh = planningMapMeshRef.current
    if (mesh && planningDisplayZ !== undefined) (mesh._group ?? mesh).position.y = planningDisplayZ
  }, [planningDisplayZ, planningMap, planningMapVisible])

  // Native control-risk grid from the field navigation endpoint.
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    if (layers.nativeTraversability && nativeTraversabilityState.status === 'ready') {
      const { event } = nativeTraversabilityState
      const transform = `${event.grid_b64}:${event.reset_epoch}:${event.sequence}`
      if (nativeTraversabilityMeshRef.current?.userData.transform === transform) return
    }
    disposeGroupedMesh(scene, nativeTraversabilityMeshRef.current)
    nativeTraversabilityMeshRef.current = null
    if (!layers.nativeTraversability || nativeTraversabilityState.status !== 'ready') return
    const mesh = createNativeTraversabilityLayer(nativeTraversabilityState)
    if (!mesh) return
    const { event } = nativeTraversabilityState
    mesh.userData.transform = `${event.grid_b64}:${event.reset_epoch}:${event.sequence}`
    scene.add(mesh._group ?? mesh)
    nativeTraversabilityMeshRef.current = mesh
  }, [nativeTraversabilityState, layers.nativeTraversability])

  useEffect(() => {
    const mesh = nativeTraversabilityMeshRef.current
    if (mesh && riskDisplayZ !== undefined) (mesh._group ?? mesh).position.y = riskDisplayZ
  }, [riskDisplayZ, nativeTraversabilityState, layers.nativeTraversability])

  // Native endpoint diagnostics are read-only and have no path back into
  // planning or control. Rebuild only at the explicitly enabled low poll rate.
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return

    disposeLocalPlannerDiagnosticLayer(scene, localPlannerRef.current)
    localPlannerRef.current = null
    if (!layers.localPlanner || (safetyView === 'slice' && !robotValid)) return

    const group = createLocalPlannerDiagnosticLayer(localPlannerSnapshot, safetyView === 'slice' ? robotPosRef.current.z : undefined)
    if (!group) return
    scene.add(group)
    localPlannerRef.current = group

    return () => {
      if (localPlannerRef.current === group) {
        disposeLocalPlannerDiagnosticLayer(scene, group)
        localPlannerRef.current = null
      }
    }
  }, [localPlannerSnapshot, layers.localPlanner, safetyView, robotValid])

  // Saved map cloud. Live map_scene labels have an independent lifecycle and
  // point ordering, so they cannot safely recolor or rebind this static PCD.
  // Height coloring stays stable until an identity-bound semantic artifact exists.
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    if (savedMapRef.current) {
      scene.remove(savedMapRef.current)
      savedMapRef.current.geometry.dispose()
      ;(savedMapRef.current.material as THREE.Material).dispose()
      savedMapRef.current = null
    }

    const points = createSavedMapLayer(
      savedMapFlat,
      SAVED_MAP_Z_FLOOR,
      SAVED_MAP_Z_CEIL,
      undefined,
      pointSizeRef.current,
    )
    if (!points) return
    scene.add(points)
    savedMapRef.current = points
  }, [savedMapFlat])

  useEffect(() => {
    if (savedMapRef.current) savedMapRef.current.visible = savedMapVisible
  }, [savedMapVisible, savedMapFlat])

  useEffect(() => {
    pointSizeRef.current = pointSize
    updateSavedMapPointSize(savedMapRef.current, pointSize)
  }, [pointSize])


  // ── Semantic scene graph (objects + labels) ────────────────────
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    if (sgGroupRef.current) {
      sgGroupRef.current.traverse(obj => {
        if ((obj as THREE.Mesh).geometry) (obj as THREE.Mesh).geometry.dispose()
        const mat = (obj as THREE.Mesh | THREE.Sprite).material as THREE.Material | THREE.Material[]
        if (Array.isArray(mat)) mat.forEach(m => m.dispose())
        else if (mat) mat.dispose()
      })
      scene.remove(sgGroupRef.current)
      sgGroupRef.current = null
    }
    if (!sceneGraph?.objects?.length) return
    const group = new THREE.Group()
    for (const obj of sceneGraph.objects) {
      const [tx, ty, tz] = lingtuToThree([obj.x, obj.y, obj.z ?? 0.5])
      // Sphere marker
      const sphereGeo = new THREE.SphereGeometry(0.2, 8, 8)
      const conf = Math.max(0, Math.min(1, obj.confidence ?? 0.5))
      const sphereMat = new THREE.MeshBasicMaterial({ color: new THREE.Color().setHSL(conf * 0.33, 1, 0.55) })
      const sphere = new THREE.Mesh(sphereGeo, sphereMat)
      sphere.position.set(tx, ty, tz)
      group.add(sphere)
      // Ground line
      const lineGeo = new THREE.BufferGeometry()
      lineGeo.setAttribute('position', new THREE.Float32BufferAttribute([tx, 0, tz, tx, ty, tz], 3))
      group.add(new THREE.Line(lineGeo, new THREE.LineBasicMaterial({ color: 0x44ffaa, opacity: 0.5, transparent: true })))
      // Text label sprite
      const cv = document.createElement('canvas')
      cv.width = 256; cv.height = 64
      const ctx = cv.getContext('2d')!
      ctx.fillStyle = 'rgba(8,10,24,0.82)'
      ctx.beginPath()
      ;(ctx as CanvasRenderingContext2D & { roundRect: (...args: unknown[]) => void }).roundRect(2, 2, 252, 60, 8)
      ctx.fill()
      ctx.font = 'bold 26px sans-serif'
      ctx.fillStyle = '#a5f3fc'
      ctx.textAlign = 'center'
      ctx.textBaseline = 'middle'
      ctx.fillText(obj.label, 128, 32)
      const tex = new THREE.CanvasTexture(cv)
      const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true }))
      sprite.position.set(tx, ty + 0.9, tz)
      sprite.scale.set(2.2, 0.55, 1)
      group.add(sprite)
    }
    scene.add(group)
    sgGroupRef.current = group
  }, [sceneGraph])

  // ── Click vs drag detection ────────────────────────────────────
  const mouseDownPos = useRef<{ x: number; y: number; shift: boolean } | null>(null)

  const handleMouseDown = (e: React.MouseEvent<HTMLDivElement>) => {
    mouseDownPos.current = { x: e.clientX, y: e.clientY, shift: e.shiftKey }
  }

  const handleMouseUp = (e: React.MouseEvent<HTMLDivElement>) => {
    if (!onPendingGoal && !onRelocalize) return
    const down = mouseDownPos.current
    if (!down) return
    const dx = e.clientX - down.x
    const dy = e.clientY - down.y
    // Only treat as click if mouse moved < 5px (not a drag/orbit)
    if (Math.hypot(dx, dy) >= 5) return

    const renderer = rendererRef.current
    const camera   = cameraRef.current
    const floor    = floorRef.current
    if (!renderer || !camera || !floor) return

    const rect = renderer.domElement.getBoundingClientRect()
    const ndc  = new THREE.Vector2(
      ((e.clientX - rect.left) / rect.width)  * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1,
    )
    raycaster.current.setFromCamera(ndc, camera)
    // Intersect the displayed underlay so oblique clicks retain the same XY.
    // Navigation target Z still comes from its original planning contract.
    const pickingZ = mappingObservationVisible ? observationDisplayZ
      : planningMapVisible ? planningDisplayZ : undefined
    const p = pickingZ !== undefined
      ? raycaster.current.ray.intersectPlane(
        new THREE.Plane(new THREE.Vector3(0, 1, 0), -pickingZ), new THREE.Vector3())
      : raycaster.current.intersectObject(floor)[0]?.point
    if (p) {
      const [wx, wy] = threeToLingtu([p.x, p.y, p.z])
      // Shift+click → relocalize (set initial pose); plain click → goal
      const shiftDown = (down.shift || e.shiftKey)
      if (shiftDown && onRelocalize) {
        onRelocalize(wx, wy)
      } else {
        onPendingGoal?.(wx, wy)
      }
    }
  }

  return (
    <div
      ref={mountRef}
      onMouseDown={handleMouseDown}
      onMouseUp={handleMouseUp}
      style={{ width: '100%', height: '100%', cursor: onPendingGoal ? 'crosshair' : 'grab', position: 'relative' }}
    >
      <div ref={robotLoadStatusRef} role="status"
        style={{ position: 'absolute', bottom: 12, left: 12, zIndex: 2, pointerEvents: 'none',
          color: 'var(--text)', background: 'var(--bg-layer-1)', borderRadius: 6, fontSize: 12 }} />
    </div>
  )
})
