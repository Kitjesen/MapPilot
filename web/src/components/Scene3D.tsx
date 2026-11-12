import { createSupportLayer } from './scene3d/layers/supportLayer.ts'
import type { SurfaceMap } from '../services/surfaceMap.ts'
import { scanCanStandAlone } from '../services/sceneTelemetry.ts'
import { mapProjectionDisplayZ, riskProjectionDisplayZ } from '../services/mapProjectionHeight.ts'
import type { MappingObservationState } from '../services/mappingObservation.ts'
import { createMappingObservationLayer } from './scene3d/layers/mappingObservationLayer.ts'
import type { Octomap } from '../services/octomap.ts'
import { createOctomapLayer, clipOctomapLayer } from './scene3d/layers/octomapLayer.ts'
import { setHeightColorRange } from './scene3d/layers/heightColor.ts'
import { createNoGoLayer } from './scene3d/layers/noGoLayer.ts'
import type { NoGoRegion } from '../services/noGo.ts'
import { CLOUD_HEIGHT_MIN, CLOUD_HEIGHT_MAX } from '../services/cloudHeightColor.ts'
import { pcdCameraDistance } from '../services/pcdPreview.ts'
/**
 * Scene3D — Three.js 3D map visualization
 *
 * Coordinate mapping:
 *   World (LingTu): X forward, Y left, Z up
 *   Three.js:       X right, Y up,      Z toward camera
 *   → Three.js pos = (worldX, worldZ_height, -worldY)
 */
import { useRef, useEffect, useMemo, useCallback, forwardRef, useImperativeHandle } from 'react'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { pickNavigationPoint, pickGoalHeading, type GoalPickMode } from './scene3d/goalPicking.ts'
import { createStairAnnotations, selectSavedMapBounds, type ScreenRectangle } from './scene3d/stairAnnotations.ts'
import type { Bounds, StairAnnotation, StairDraft, StairTool, XYZ } from '../types/stairs.ts'
import type { MapEditScene, MapEditView } from '../types/mapEdit.ts'
import { applyMapEditOperations, isMapCrop, mapEditCloudBounds, mapEditKeepsPoint, pointInMapEditBounds } from '../services/mapEdit.ts'
import { alignMapEditTopView, createMapEditBox, frameMapEditView, mapEditDragBounds, mapEditSelectionAnchor } from './scene3d/mapEditing.ts'
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
import { disposeGroupedMesh, interpolateCloudHeightRange, updateCloudHeightRange, visibleCloudBounds, type CloudHeightRange, type CloudHeightStyle, type GroupedMesh } from './scene3d/layers/layerUtils'
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
  focusPoint(x: number, y: number, z: number): void
  resetCamera(): void
  topView(height?: number): void
  fitMap(): void
  mapEditView(view: MapEditView): void
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
  navigationMap?: Octomap
  navigationMapRadius?: number
  noGoRegions?: readonly NoGoRegion[]
  supportMap?: SurfaceMap
  voxelScale?: number
  cloudColorRange?: { min: number; max: number } | null
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
  cloudColorMode?: 'height' | 'neutral'
  cloudMinZ?: number | null
  cloudMaxZ?: number | null
  cloudHeightStyle?: CloudHeightStyle
  pointSize:    number
  onPendingGoal?: (x: number, y: number, z?: number, mode?: GoalPickMode) => { x: number; y: number; z?: number } | void
  goalPickMode?: GoalPickMode
  onRelocalize?: (x: number, y: number) => void
  pendingGoal?:  { x: number; y: number; z?: number } | null
  pendingGoalYaw?: number
  plannedGoal?: { x: number; y: number; z: number } | null
  goalHeadingEditing?: boolean
  onGoalHeadingChange?: (yaw: number | undefined) => void
  onGoalHeadingComplete?: () => void
  pendingGoalLabel?: string
  previewPath?: PathPoint[]
  routeDraftPoints?: Array<{ x: number; y: number; z: number }>
  stairAnnotations?: StairAnnotation[]
  stairDraft?: StairDraft | null
  stairTool?: StairTool
  onStairBounds?: (bounds: Bounds) => void
  onStairPoint?: (point: XYZ) => void
  mapEdit?: MapEditScene | null
  onMapEditBounds?: (bounds: Bounds) => void
  onMapEditAnchor?: (anchor: { x: number; y: number } | null) => void
}

const LIVE_SCAN_COLOR = 0xf1f5f9
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

function makeGoalLabel(title: string, color: string, textures: THREE.CanvasTexture[]) {
  const label = document.createElement('canvas')
  label.width = 384; label.height = 64
  const context = label.getContext('2d')
  if (!context) return
  const uiFont = getComputedStyle(document.documentElement).getPropertyValue('--font-ui').trim() || 'sans-serif'
  context.font = `500 26px ${uiFont}`; context.textAlign = 'center'; context.textBaseline = 'middle'
  const labelWidth = Math.min(372, context.measureText(title).width + 28)
  context.fillStyle = 'rgba(20, 28, 36, 0.94)'
  context.beginPath(); context.roundRect((384 - labelWidth) / 2, 8, labelWidth, 48, 16); context.fill()
  context.fillStyle = color; context.fillText(title, 192, 32)
  const texture = new THREE.CanvasTexture(label)
  textures.push(texture)
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, depthTest: false, depthWrite: false }))
  sprite.position.y = 0.38; sprite.center.set(.5, -.25); sprite.scale.set(1.6, 0.267, 1); sprite.renderOrder = 52
  return sprite
}

export const Scene3D = forwardRef<Scene3DHandle, Scene3DProps>(function Scene3D(
  { cloud, scanCloud, savedMapFlat, savedMapFrameId, savedMapEpoch, savedMapVisible = true, navigationMap, navigationMapRadius, noGoRegions, supportMap, voxelScale = 1, cloudColorRange, mappingObservation, mappingObservationVisible = false, scanVisible = true, elevationState, nativeTraversabilityState, sceneGraph, robotX, robotY, robotZ = 0, orientation, poseStampS, poseEpoch, robotModel, jointTelemetry, followRobot = false, robotValid, yaw, trail, path, localPath, localPlannerSnapshot, safetyEnvelope, safetyView = 'slice', layers, pointSize, cloudColorMode = 'height', cloudMinZ, cloudMaxZ, cloudHeightStyle = 'clip', onPendingGoal, goalPickMode = 'plane', onRelocalize, pendingGoal, pendingGoalYaw, plannedGoal = null, goalHeadingEditing = false, onGoalHeadingChange, onGoalHeadingComplete, pendingGoalLabel = '待确认目标', previewPath, routeDraftPoints = [], stairAnnotations = [], stairDraft, stairTool = null, onStairBounds, onStairPoint, mapEdit = null, onMapEditBounds, onMapEditAnchor },
  ref,
) {
  const mountRef   = useRef<HTMLDivElement>(null)
  const sceneRef   = useRef<THREE.Scene | null>(null)
  const cameraRef  = useRef<THREE.PerspectiveCamera | null>(null)
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null)
  const octomapRef = useRef<THREE.InstancedMesh | null>(null)
  const noGoRef = useRef<THREE.Group | null>(null)
  const controlsRef = useRef<OrbitControls | null>(null)
  const rafRef     = useRef(0)
  const stairToolRef = useRef(stairTool)
  const stairSelectionRef = useRef<HTMLDivElement>(null)
  const stairPointerRef = useRef<{ pointerId: number; tool: 'bounds' | 'lower' | 'upper'; rectangle: ScreenRectangle } | null>(null)
  const goalHeadingPointerRef = useRef<{ pointerId: number; yaw: number | undefined; heading: number | undefined;
    goal: { x: number; y: number; z?: number }; x: number; y: number; element: HTMLDivElement; controlsEnabled: boolean } | null>(null)
  const clearGoalHeadingPointer = useCallback(() => {
    const pointer = goalHeadingPointerRef.current
    goalHeadingPointerRef.current = null
    if (!pointer) return
    if (pointer.element.hasPointerCapture(pointer.pointerId)) pointer.element.releasePointerCapture(pointer.pointerId)
    if (controlsRef.current) controlsRef.current.enabled = pointer.controlsEnabled
  }, [])
  const goalGestureEnabled = !!onPendingGoal && !!onGoalHeadingChange && !mapEdit && !stairTool
  useEffect(() => clearGoalHeadingPointer, [clearGoalHeadingPointer, goalGestureEnabled])
  useEffect(() => {
    const pointer = goalHeadingPointerRef.current
    if (pointer && pointer.goal !== pendingGoal) clearGoalHeadingPointer()
  }, [pendingGoal, clearGoalHeadingPointer])
  const mapEditPointerRef = useRef<{ pointerId: number; rectangle: ScreenRectangle } | null>(null)
  const mapEditRef = useRef(mapEdit)
  const mapEditAnchorCallbackRef = useRef(onMapEditAnchor)
  const mapEditAnchorRef = useRef<{ x: number; y: number } | null>(null)
  const mapEditSourceBounds = useMemo(() => mapEditCloudBounds(savedMapFlat ?? []), [savedMapFlat])
  const mapEditBoundsRef = useRef(mapEditSourceBounds)
  const editingMap = mapEdit != null
  const editSelection = mapEdit?.selection
  const editOperations = mapEdit?.operations
  const editView = mapEdit?.view
  const editShowDeleted = mapEdit?.showDeleted
  const editPreview = mapEdit?.preview
  const draftOperations = editView === 'draft' ? editOperations : undefined
  const displayMapFlat = useMemo(() => {
    if (!draftOperations?.length || !savedMapFlat) return savedMapFlat
    if (draftOperations.some(edit => !isMapCrop(edit))) return editPreview?.preview_points ?? []
    return applyMapEditOperations(savedMapFlat, draftOperations)
  }, [savedMapFlat, draftOperations, editPreview])
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
  const pendingGoalArrowRef = useRef<THREE.ArrowHelper | null>(null)
  const elevationMeshRef = useRef<GroupedMesh | null>(null)
  const mappingObservationRef = useRef<GroupedMesh | null>(null)
  const nativeTraversabilityMeshRef = useRef<GroupedMesh | null>(null)
  const localPlannerRef = useRef<THREE.Group | null>(null)
  const gridRef        = useRef<THREE.GridHelper | null>(null)
  const floorRef   = useRef<THREE.Mesh | null>(null)
  const liveCloudRef   = useRef<THREE.Points | null>(null)
  const scanCloudRef   = useRef<THREE.Points | null>(null)
  const savedMapRef    = useRef<THREE.Points | null>(null)
  const cloudHeightRef = useRef({
    current: { minZ: null, maxZ: null } as CloudHeightRange,
    from: { minZ: null, maxZ: null } as CloudHeightRange,
    target: { minZ: null, maxZ: null } as CloudHeightRange,
    startedAt: 0,
    moving: false,
    poseEpoch,
    savedMapEpoch,
    style: cloudHeightStyle,
  })
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
  const underlayDisplayZ = mappingObservationVisible ? observationDisplayZ : undefined
  const riskDisplayZ = nativeTraversabilityState.status === 'ready'
    ? riskProjectionDisplayZ(nativeTraversabilityState.event.origin[2], robotZ, robotModel, robotValid, underlayDisplayZ)
    : undefined

  // ── Expose resetCamera ──────────────────────────────────────────
  useImperativeHandle(ref, () => ({
    mapEditView(view) {
      const camera = cameraRef.current
      const controls = controlsRef.current
      const bounds = mapEditRef.current?.selection ?? mapEditBoundsRef.current
      if (!camera || !controls || !bounds) return
      frameMapEditView(camera, controls.target, bounds, view)
      controls.update()
    },
    focusPoint(x, y, z) {
      const camera = cameraRef.current
      const controls = controlsRef.current
      if (!camera || !controls) return
      const target = new THREE.Vector3(...lingtuToThree([x, y, z]))
      camera.position.add(target.clone().sub(controls.target))
      controls.target.copy(target)
      controls.update()
    },
    fitMap() {
      const camera = cameraRef.current
      const controls = controlsRef.current
      const raster = mappingObservationRef.current
      const points = raster ? raster._group ?? raster
        : octomapRef.current ?? (savedMapRef.current?.visible ? savedMapRef.current : liveCloudRef.current)
      if (!camera || !controls || !points) return
      const bounds = points instanceof THREE.Points
        ? visibleCloudBounds(points) : new THREE.Box3().setFromObject(points)
      if (bounds.isEmpty()) return
      const center = bounds.getCenter(new THREE.Vector3())
      const size = bounds.getSize(new THREE.Vector3())
      const radius = size.length() / 2
      const distance = Math.max(controls.minDistance, pcdCameraDistance(radius, camera.aspect))
      controls.maxDistance = Math.max(controls.maxDistance, distance * 1.2)
      camera.far = Math.max(camera.far, distance + radius * 2)
      camera.updateProjectionMatrix()
      camera.position.set(center.x, center.y + distance, center.z + 0.001)
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
    let w = mount.clientWidth, h = mount.clientHeight

    const scene = new THREE.Scene()
    sceneRef.current = scene

    const camera = new THREE.PerspectiveCamera(48, w / h, 0.02, 500)
    camera.position.set(-3.8, 3.2, 4.8)
    camera.lookAt(0, 0, 0)
    cameraRef.current = camera

    const renderer = new THREE.WebGLRenderer({ antialias: true })
    renderer.localClippingEnabled = true
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
    renderer.setSize(w, h)
    renderer.toneMapping = THREE.ACESFilmicToneMapping
    renderer.toneMappingExposure = 1.15
    mount.appendChild(renderer.domElement)
    rendererRef.current = renderer

    const controls = new OrbitControls(camera, renderer.domElement)
    // Left drag selects a navigation goal; camera gestures must remain available.
    controls.mouseButtons.RIGHT = THREE.MOUSE.ROTATE
    controls.mouseButtons.MIDDLE = THREE.MOUSE.PAN
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
    grid.position.y = -0.02
    scene.add(grid)
    gridRef.current = grid

    grid.material.transparent = true
    const applyTheme = () => {
      scene.background = new THREE.Color(getComputedStyle(mount).getPropertyValue('--bg-layer-1').trim())
      grid.material.opacity = document.documentElement.dataset.theme === 'dark' ? 0.18 : 0.55
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
      const height = cloudHeightRef.current
      if (height.moving) {
        const progress = (performance.now() - height.startedAt) / 200
        height.current = interpolateCloudHeightRange(height.from, height.target, progress)
        height.moving = progress < 1
        for (const points of [savedMapRef.current, liveCloudRef.current, scanCloudRef.current]) {
          updateCloudHeightRange(points, height.current.minZ, height.current.maxZ, height.style)
        }
      }
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
        if (followRef.current && !stairToolRef.current && !mapEditRef.current && !goalHeadingPointerRef.current) {
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
      if ((!stairToolRef.current || mapEditRef.current) && !mapEditPointerRef.current && !goalHeadingPointerRef.current) controls.update()
      const selection = mapEditRef.current?.selection
      const onAnchor = mapEditAnchorCallbackRef.current
      if (onAnchor) {
        const anchor = selection ? mapEditSelectionAnchor(camera,
          { width: mount.clientWidth, height: mount.clientHeight }, selection) : null
        const previous = mapEditAnchorRef.current
        if (anchor ? !previous || Math.abs(anchor.x - previous.x) > 2 || Math.abs(anchor.y - previous.y) > 2 : previous) {
          mapEditAnchorRef.current = anchor
          onAnchor(anchor)
        }
      }
      renderer.render(scene, camera)
    }
    animate()

    const ro = new ResizeObserver(() => {
      const nw = mount.clientWidth, nh = mount.clientHeight
      w = nw; h = nh
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

  useEffect(() => {
    stairToolRef.current = stairTool
    if (controlsRef.current) controlsRef.current.enabled = !stairTool
    stairPointerRef.current = null
    if (stairSelectionRef.current) stairSelectionRef.current.style.display = 'none'
  }, [stairTool])

  useEffect(() => {
    mapEditRef.current = mapEdit
    mapEditBoundsRef.current = mapEditSourceBounds
    mapEditAnchorCallbackRef.current = onMapEditAnchor
  }, [mapEdit, mapEditSourceBounds, onMapEditAnchor])

  useEffect(() => {
    const controls = controlsRef.current
    if (!controls) return
    controls.maxPolarAngle = editingMap ? Math.PI / 2 : Math.PI * 0.475
    controls.enabled = editingMap || !stairTool
    controls.enableRotate = !mapEdit?.selecting
    controls.enablePan = !mapEdit?.selecting
    controls.enableDamping = !mapEdit?.selecting
    mapEditPointerRef.current = null
    if (stairSelectionRef.current) stairSelectionRef.current.style.display = 'none'
  }, [editingMap, mapEdit?.selecting, stairTool])

  useEffect(() => {
    const edit = mapEditRef.current
    if (!edit?.selecting) return
    const camera = cameraRef.current
    const controls = controlsRef.current
    if (camera && controls) {
      const target = controls.target.clone()
      const distance = camera.position.distanceTo(target)
      // Flush pending orbit damping before restoring the user's target and zoom.
      controls.update()
      controls.target.copy(target)
      alignMapEditTopView(camera, controls.target, distance)
      controls.update()
    }
    // Changing a selection or original/draft must not reset the user's camera.
  }, [mapEdit?.selecting])

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
      color: 0xffffff,
      opacity: 0.98,
      pointSizeScale: 1.5,
      renderOrder: 6,
      vertexColors: true,
      heightColors: true,
    })
    if (liveCloudRef.current) (liveCloudRef.current.material as THREE.PointsMaterial).toneMapped = false
  }, [cloud, layers.cloud, liveCloudFrameAllowed, pointSize])

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
      : mapProjectionDisplayZ(0, robotZ, robotModel, robotValid)
    if (gridRef.current) gridRef.current.position.y = z - .012
    if (floorRef.current) floorRef.current.position.y = z - .012
  }, [mappingObservationVisible, observationDisplayZ, robotZ, robotModel, robotValid])

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
      new THREE.SphereGeometry(0.03, 12, 8),
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
    const center = new THREE.Mesh(
      new THREE.CircleGeometry(0.03, 24),
      new THREE.MeshBasicMaterial({ color: 0xe8edf1, depthTest: false, depthWrite: false }),
    )
    center.rotation.x = -Math.PI / 2
    center.position.y = 0.006
    center.renderOrder = 51
    marker.add(center)
    const arrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, 0.025, 0), 0.25, 0xe8edf1, 0.07, 0.05,
    )
    for (const part of [arrow.line, arrow.cone]) {
      const materials = Array.isArray(part.material) ? part.material : [part.material]
      materials.forEach(material => { material.depthTest = false; material.depthWrite = false })
    }
    arrow.line.renderOrder = 51
    arrow.cone.renderOrder = 51
    arrow.visible = false
    pendingGoalArrowRef.current = arrow
    marker.add(arrow)
    const textures: THREE.CanvasTexture[] = []
    const endpointOffset = plannedGoal && typeof pendingGoal.z === 'number'
      ? new THREE.Vector3(plannedGoal.x - pendingGoal.x, plannedGoal.z - pendingGoal.z, pendingGoal.y - plannedGoal.y) : null
    const separateEndpoint = endpointOffset && endpointOffset.length() >= .001
    if (separateEndpoint) {
      const targetLabel = makeGoalLabel(pendingGoalLabel, '#e8edf1', textures)
      if (targetLabel) marker.add(targetLabel)
    }
    if (separateEndpoint && endpointOffset) {
      const endpoint = new THREE.Group()
      endpoint.position.copy(endpointOffset)
      const dot = new THREE.Mesh(new THREE.SphereGeometry(.03, 12, 8),
        new THREE.MeshBasicMaterial({ color: 0x67d9df, depthTest: false, depthWrite: false }))
      dot.renderOrder = 53; endpoint.add(dot)
      const endpointLabel = makeGoalLabel('规划终点', '#67d9df', textures)
      // Screen-space anchors keep both labels clear of the points even in top view.
      if (endpointLabel) { endpointLabel.center.y = -1.5; endpoint.add(endpointLabel) }
      marker.add(endpoint)
      const connector = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, .015, 0), endpointOffset]),
        new THREE.LineDashedMaterial({ color: 0x67d9df, dashSize: .06, gapSize: .035, depthTest: false, depthWrite: false }))
      connector.computeLineDistances(); connector.renderOrder = 52; marker.add(connector)
    }
    scene.add(marker)
    return () => { pendingGoalArrowRef.current = null; removeFrom(scene, marker); textures.forEach(texture => texture.dispose()) }
  }, [pendingGoal, pendingGoalLabel, plannedGoal, underlayDisplayZ])

  useEffect(() => {
    const arrow = pendingGoalArrowRef.current
    if (!arrow) return
    arrow.visible = pendingGoalYaw !== undefined
    if (pendingGoalYaw !== undefined) arrow.setDirection(new THREE.Vector3(Math.cos(pendingGoalYaw), 0, -Math.sin(pendingGoalYaw)))
  }, [pendingGoalYaw, pendingGoal, pendingGoalLabel, plannedGoal, underlayDisplayZ])

  // Route draft markers show click order only. The native planner owns the
  // executable path after the operator starts the route.
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene || routeDraftPoints.length === 0) return
    const group = new THREE.Group()
    const textures: THREE.CanvasTexture[] = []

    routeDraftPoints.forEach((point, index) => {
      const marker = new THREE.Group()
      marker.position.set(...lingtuToThree([point.x, point.y, point.z + 0.06]))
      const dot = new THREE.Mesh(
        new THREE.SphereGeometry(0.09, 14, 10),
        new THREE.MeshBasicMaterial({ color: 0xd7e2ed, depthTest: false }),
      )
      dot.renderOrder = 48
      marker.add(dot)

      const canvas = document.createElement('canvas')
      canvas.width = 96
      canvas.height = 96
      const context = canvas.getContext('2d')
      if (context) {
        context.fillStyle = '#1e2935'
        context.beginPath()
        context.arc(48, 48, 42, 0, Math.PI * 2)
        context.fill()
        context.strokeStyle = '#e8eef4'
        context.lineWidth = 4
        context.stroke()
        context.fillStyle = '#f8fafc'
        context.font = '600 42px sans-serif'
        context.textAlign = 'center'
        context.textBaseline = 'middle'
        context.fillText(String(index + 1), 48, 50)
        const texture = new THREE.CanvasTexture(canvas)
        textures.push(texture)
        const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, depthTest: false }))
        sprite.position.y = 0.28
        sprite.scale.set(0.28, 0.28, 1)
        sprite.renderOrder = 49
        marker.add(sprite)
      }
      group.add(marker)
    })

    if (routeDraftPoints.length > 1) {
      const geometry = new THREE.BufferGeometry().setFromPoints(routeDraftPoints.map(point =>
        new THREE.Vector3(...lingtuToThree([point.x, point.y, point.z + 0.065]))))
      const line = new THREE.Line(geometry, new THREE.LineDashedMaterial({
        color: 0xd7e2ed,
        dashSize: 0.1,
        gapSize: 0.08,
        opacity: 0.8,
        transparent: true,
        depthTest: false,
      }))
      line.computeLineDistances()
      line.renderOrder = 47
      group.add(line)
    }

    scene.add(group)
    return () => {
      removeFrom(scene, group)
      textures.forEach(texture => texture.dispose())
    }
  }, [routeDraftPoints])

  useEffect(() => {
    const scene = sceneRef.current
    if (!scene || !previewPath || previewPath.length < 2) return
    const geometry = new THREE.BufferGeometry().setFromPoints(previewPath.map(point =>
      new THREE.Vector3(...lingtuToThree([point.x, point.y, (point.z ?? 0) + 0.06]))))
    const line = new THREE.Line(geometry, new THREE.LineDashedMaterial({
      color: 0xe8edf1, dashSize: 0.12, gapSize: 0.08, depthTest: false,
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
  // Color mode changes only the display material, never the source geometry.
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
      displayMapFlat,
      SAVED_MAP_Z_FLOOR,
      SAVED_MAP_Z_CEIL,
      undefined,
      pointSizeRef.current,
    )
    if (!points) return
    scene.add(points)
    savedMapRef.current = points
  }, [displayMapFlat])

  useEffect(() => {
    if (savedMapRef.current) savedMapRef.current.visible = savedMapVisible
  }, [savedMapVisible, displayMapFlat])

  const voxelCenterX = navigationMap ? Math.round((robotValid ? robotX : (navigationMap.bounds.min[0] + navigationMap.bounds.max[0]) / 2) / 2) * 2 : 0
  const voxelCenterY = navigationMap ? Math.round((robotValid ? robotY : (navigationMap.bounds.min[1] + navigationMap.bounds.max[1]) / 2) / 2) * 2 : 0
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene || !navigationMap) return
    const mesh = createOctomapLayer(navigationMap, voxelCenterX, voxelCenterY, cloudColorMode, voxelScale, navigationMapRadius)
    if (!mesh) return
    scene.add(mesh)
    octomapRef.current = mesh
    return () => { mesh.dispose(); removeFrom(scene, mesh); octomapRef.current = null }
  }, [navigationMap, voxelCenterX, voxelCenterY, cloudColorMode, voxelScale, navigationMapRadius])

  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    removeFrom(scene, noGoRef.current)
    noGoRef.current = noGoRegions?.length ? createNoGoLayer(noGoRegions) : null
    if (noGoRef.current) scene.add(noGoRef.current)
    return () => { removeFrom(scene, noGoRef.current); noGoRef.current = null }
  }, [noGoRegions])

  useEffect(() => {
    const scene = sceneRef.current
    if (!scene || !supportMap) return
    const layer = createSupportLayer(supportMap)
    scene.add(layer)
    return () => removeFrom(scene, layer)
  }, [supportMap])

  useEffect(() => {
    if (octomapRef.current) clipOctomapLayer(octomapRef.current, cloudMinZ, cloudMaxZ)
  }, [navigationMap, voxelCenterX, voxelCenterY, cloudColorMode, voxelScale, cloudMinZ, cloudMaxZ])

  // Local material planes affect only the clouds, never routes or robot geometry.
  // Changing the height range updates existing GPU layers without rewriting source points.
  useEffect(() => {
    const height = cloudHeightRef.current
    const target = {
      minZ: !editingMap && cloudMinZ != null && Number.isFinite(cloudMinZ) ? cloudMinZ : null,
      maxZ: !editingMap && cloudMaxZ != null && Number.isFinite(cloudMaxZ) ? cloudMaxZ : null,
    }
    const epochChanged = height.poseEpoch !== poseEpoch || height.savedMapEpoch !== savedMapEpoch
    const styleChanged = height.style !== cloudHeightStyle
    if (epochChanged || styleChanged || target.minZ !== height.target.minZ || target.maxZ !== height.target.maxZ) {
      const snap = epochChanged || styleChanged || (target.minZ == null) !== (height.current.minZ == null)
        || (target.maxZ == null) !== (height.current.maxZ == null)
        || window.matchMedia('(prefers-reduced-motion: reduce)').matches
      height.from = height.current
      height.target = target
      height.startedAt = performance.now()
      height.moving = !snap
      if (snap) height.current = target
      height.poseEpoch = poseEpoch
      height.savedMapEpoch = savedMapEpoch
      height.style = cloudHeightStyle
    }
    for (const points of [savedMapRef.current, liveCloudRef.current, scanCloudRef.current]) {
      updateCloudHeightRange(points, height.current.minZ, height.current.maxZ, height.style)
    }
  }, [cloudMinZ, cloudMaxZ, cloudHeightStyle, poseEpoch, savedMapEpoch, displayMapFlat, cloud, scanCloud, layers.cloud, scanVisible, liveCloudFrameAllowed, scanFrameAligned, pointSize, editingMap])

  useEffect(() => {
    pointSizeRef.current = pointSize
    updateSavedMapPointSize(savedMapRef.current, pointSize)
  }, [pointSize])

  useEffect(() => {
    for (const points of [savedMapRef.current, liveCloudRef.current]) {
      if (!points) continue
      const material = points.material as THREE.PointsMaterial
      material.vertexColors = cloudColorMode === 'height'
      material.color.setHex(cloudColorMode === 'height' ? 0xffffff : 0xc3c6c9)
      material.needsUpdate = true
    }
  }, [cloudColorMode, displayMapFlat, cloud, layers.cloud, liveCloudFrameAllowed, pointSize])

  useEffect(() => {
    for (const object of [savedMapRef.current, liveCloudRef.current, octomapRef.current]) {
      if (!object) continue
      setHeightColorRange(object.material as THREE.Material,
        cloudColorRange?.min ?? CLOUD_HEIGHT_MIN, cloudColorRange?.max ?? CLOUD_HEIGHT_MAX,
        cloudColorMode === 'height')
    }
  }, [cloudColorRange, cloudColorMode, displayMapFlat, cloud, layers.cloud, liveCloudFrameAllowed, pointSize,
    navigationMap, voxelCenterX, voxelCenterY, voxelScale])

  useEffect(() => {
    const scene = sceneRef.current
    if (!scene || !editingMap || !savedMapVisible) return
    const group = new THREE.Group()
    const selection = editSelection
    if (selection) group.add(createMapEditBox(selection))
    const displayed = savedMapRef.current?.geometry.getAttribute('position')
    if (selection && displayed) {
      const selected: number[] = []
      for (let i = 0; i < displayed.count; i++) {
        const x = displayed.getX(i), z = displayed.getY(i), y = -displayed.getZ(i)
        if (pointInMapEditBounds(x, y, z, selection)) selected.push(x, y, z)
      }
      const highlight = createSavedMapLayer(selected, SAVED_MAP_Z_FLOOR, SAVED_MAP_Z_CEIL, undefined, pointSize * 1.15)
      if (highlight) {
        const material = highlight.material as THREE.PointsMaterial
        material.vertexColors = false
        material.color.setHex(0xe6b96d)
        material.depthTest = false
        highlight.renderOrder = 54
        group.add(highlight)
      }
    }
    if (editView === 'draft' && editShowDeleted && editOperations?.length && savedMapFlat) {
      const removed: number[] = editPreview ? [...editPreview.preview_removed] : []
      if (editOperations.every(isMapCrop)) for (let i = 0; i + 2 < savedMapFlat.length; i += 3) {
        if (!mapEditKeepsPoint(savedMapFlat[i], savedMapFlat[i + 1], savedMapFlat[i + 2], editOperations)) {
          removed.push(savedMapFlat[i], savedMapFlat[i + 1], savedMapFlat[i + 2])
        }
      }
      const deleted = createSavedMapLayer(removed, SAVED_MAP_Z_FLOOR, SAVED_MAP_Z_CEIL, undefined, pointSize)
      if (deleted) {
        const material = deleted.material as THREE.PointsMaterial
        material.vertexColors = false
        material.color.setHex(0xd79883)
        material.opacity = 0.25
        group.add(deleted)
      }
    }
    scene.add(group)
    return () => removeFrom(scene, group)
  }, [editingMap, editSelection, editOperations, editView, editShowDeleted, editPreview, displayMapFlat, savedMapFlat, savedMapVisible, pointSize])



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

  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    const group = createStairAnnotations(stairAnnotations, stairDraft)
    const textures: THREE.CanvasTexture[] = []
    group.traverse(object => {
      if (object.name !== 'stair-lower' && object.name !== 'stair-upper') return
      const canvas = document.createElement('canvas')
      canvas.width = 160; canvas.height = 48
      const context = canvas.getContext('2d')
      if (!context) return
      const uiFont = getComputedStyle(document.documentElement).getPropertyValue('--font-ui').trim() || 'sans-serif'
      context.font = `600 22px ${uiFont}`
      context.textAlign = 'center'; context.textBaseline = 'middle'
      context.fillStyle = 'rgba(16,21,29,0.88)'
      context.beginPath(); context.roundRect(22, 7, 116, 34, 6); context.fill()
      context.fillStyle = '#d9eaf6'
      context.fillText(object.name === 'stair-lower' ? '下入口' : '上入口', 80, 24)
      const texture = new THREE.CanvasTexture(canvas)
      textures.push(texture)
      const label = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false }))
      label.name = `${object.name}-label`
      label.position.y = 0.2
      label.scale.set(0.65, 0.195, 1)
      label.renderOrder = 45
      object.add(label)
    })
    scene.add(group)
    return () => { removeFrom(scene, group); textures.forEach(texture => texture.dispose()) }
  }, [stairAnnotations, stairDraft])

  const clearMapEditPointer = (event: React.PointerEvent<HTMLDivElement>) => {
    mapEditPointerRef.current = null
    mouseDownPos.current = null
    if (stairSelectionRef.current) stairSelectionRef.current.style.display = 'none'
    if (event.currentTarget.hasPointerCapture(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId)
    if (controlsRef.current) controlsRef.current.enabled = mapEdit != null || !stairTool
  }

  const handleMapEditPointerDown = (event: React.PointerEvent<HTMLDivElement>) => {
    if (!mapEdit?.selecting) return
    event.stopPropagation()
    event.preventDefault()
    if (event.button !== 0 || mapEditPointerRef.current) return
    const renderer = rendererRef.current
    const camera = cameraRef.current
    const controls = controlsRef.current
    if (!renderer || !camera || !controls || !mapEditSourceBounds) return
    // Selection remains vertical even if the user has orbited since enabling it.
    alignMapEditTopView(camera, controls.target)
    controls.enabled = false
    const rect = renderer.domElement.getBoundingClientRect()
    const x = event.clientX - rect.left, y = event.clientY - rect.top
    mapEditPointerRef.current = { pointerId: event.pointerId, rectangle: { x0: x, y0: y, x1: x, y1: y } }
    event.currentTarget.setPointerCapture(event.pointerId)
    if (stairSelectionRef.current) Object.assign(stairSelectionRef.current.style,
      { display: 'block', left: `${x}px`, top: `${y}px`, width: '0px', height: '0px' })
  }

  const handleMapEditPointerMove = (event: React.PointerEvent<HTMLDivElement>) => {
    const pointer = mapEditPointerRef.current
    const renderer = rendererRef.current
    if (!pointer || pointer.pointerId !== event.pointerId || !renderer) return
    event.stopPropagation()
    const rect = renderer.domElement.getBoundingClientRect()
    const selection = pointer.rectangle
    selection.x1 = Math.max(0, Math.min(rect.width, event.clientX - rect.left))
    selection.y1 = Math.max(0, Math.min(rect.height, event.clientY - rect.top))
    if (stairSelectionRef.current) Object.assign(stairSelectionRef.current.style, {
      left: `${Math.min(selection.x0, selection.x1)}px`, top: `${Math.min(selection.y0, selection.y1)}px`,
      width: `${Math.abs(selection.x1 - selection.x0)}px`, height: `${Math.abs(selection.y1 - selection.y0)}px`,
    })
  }

  const handleMapEditPointerUp = (event: React.PointerEvent<HTMLDivElement>) => {
    const pointer = mapEditPointerRef.current
    if (!pointer || pointer.pointerId !== event.pointerId) return
    event.stopPropagation()
    event.preventDefault()
    handleMapEditPointerMove(event)
    clearMapEditPointer(event)
    const renderer = rendererRef.current
    const camera = cameraRef.current
    if (!mapEdit?.selecting || !renderer || !camera || !mapEditSourceBounds) return
    const bounds = mapEditDragBounds(camera, renderer.domElement.getBoundingClientRect(), pointer.rectangle, mapEditSourceBounds)
    if (bounds) onMapEditBounds?.(bounds)
  }

  // Annotation tools capture before OrbitControls receives pointer events.
  const handleStairPointerDown = (event: React.PointerEvent<HTMLDivElement>) => {
    if (!stairTool) return
    event.stopPropagation()
    event.preventDefault()
    if (controlsRef.current) controlsRef.current.enabled = false
    if (event.button !== 0) return
    const renderer = rendererRef.current
    if (!renderer || !savedMapFlat || !savedMapRef.current?.visible) return
    const rect = renderer.domElement.getBoundingClientRect()
    const x = event.clientX - rect.left
    const y = event.clientY - rect.top
    stairPointerRef.current = { pointerId: event.pointerId, tool: stairTool, rectangle: { x0: x, y0: y, x1: x, y1: y } }
    event.currentTarget.setPointerCapture(event.pointerId)
    if (stairTool === 'bounds' && stairSelectionRef.current) {
      Object.assign(stairSelectionRef.current.style, { display: 'block', left: `${x}px`, top: `${y}px`, width: '0px', height: '0px' })
    }
  }

  const handleStairPointerMove = (event: React.PointerEvent<HTMLDivElement>) => {
    const pointer = stairPointerRef.current
    const renderer = rendererRef.current
    if (!pointer || pointer.pointerId !== event.pointerId || !renderer) return
    event.stopPropagation()
    const rect = renderer.domElement.getBoundingClientRect()
    const selection = pointer.rectangle
    selection.x1 = Math.max(0, Math.min(rect.width, event.clientX - rect.left))
    selection.y1 = Math.max(0, Math.min(rect.height, event.clientY - rect.top))
    if (pointer.tool === 'bounds' && stairSelectionRef.current) {
      Object.assign(stairSelectionRef.current.style, {
        left: `${Math.min(selection.x0, selection.x1)}px`, top: `${Math.min(selection.y0, selection.y1)}px`,
        width: `${Math.abs(selection.x1 - selection.x0)}px`, height: `${Math.abs(selection.y1 - selection.y0)}px`,
      })
    }
  }

  const clearStairPointer = (event: React.PointerEvent<HTMLDivElement>) => {
    stairPointerRef.current = null
    if (stairSelectionRef.current) stairSelectionRef.current.style.display = 'none'
    if (event.currentTarget.hasPointerCapture(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId)
  }

  const handleStairPointerUp = (event: React.PointerEvent<HTMLDivElement>) => {
    if (!stairTool) return
    event.stopPropagation()
    event.preventDefault()
    const pointer = stairPointerRef.current
    const renderer = rendererRef.current
    const camera = cameraRef.current
    const points = savedMapRef.current
    clearStairPointer(event)
    if (!pointer || pointer.pointerId !== event.pointerId || pointer.tool !== stairTool || !renderer || !camera || !points?.visible || !savedMapFlat) return
    const rect = renderer.domElement.getBoundingClientRect()
    const selection = pointer.rectangle
    selection.x1 = Math.max(0, Math.min(rect.width, event.clientX - rect.left))
    selection.y1 = Math.max(0, Math.min(rect.height, event.clientY - rect.top))
    if (pointer.tool === 'bounds') {
      if (Math.abs(selection.x1 - selection.x0) < 5 || Math.abs(selection.y1 - selection.y0) < 5) return
      const bounds = selectSavedMapBounds(savedMapFlat, points, camera, rect, selection)
      if (bounds) onStairBounds?.(bounds)
      return
    }
    if (Math.hypot(selection.x1 - selection.x0, selection.y1 - selection.y0) >= 5) return
    raycaster.current.setFromCamera(new THREE.Vector2(selection.x1 / rect.width * 2 - 1, 1 - selection.y1 / rect.height * 2), camera)
    const point = pickNavigationPoint(raycaster.current, [points], robotZ, 'surface')
    if (point) {
      const [x, y, z] = threeToLingtu([point.x, point.y, point.z])
      onStairPoint?.({ x, y, z })
    }
  }

  const navigationPointAt = (clientX: number, clientY: number, mode: GoalPickMode) => {
    const camera = cameraRef.current, renderer = rendererRef.current
    if (!camera || !renderer) return null
    const rect = renderer.domElement.getBoundingClientRect()
    raycaster.current.setFromCamera(new THREE.Vector2(
      (clientX - rect.left) / rect.width * 2 - 1, 1 - (clientY - rect.top) / rect.height * 2,
    ), camera)
    const pickingZ = mappingObservationVisible ? observationDisplayZ ?? robotZ : robotZ
    const pickableClouds = mappingObservationVisible ? [] : [savedMapRef.current, liveCloudRef.current, scanCloudRef.current]
      .filter((object): object is THREE.Points => Boolean(object?.visible))
    return pickNavigationPoint(raycaster.current, pickableClouds, pickingZ, mode, octomapRef.current)
  }

  const handleGoalHeadingPointerMove = (event: React.PointerEvent<HTMLDivElement>) => {
    const pointer = goalHeadingPointerRef.current
    const camera = cameraRef.current, renderer = rendererRef.current
    if (!pointer || pointer.pointerId !== event.pointerId || !camera || !renderer) return
    event.preventDefault(); event.stopPropagation()
    if (Math.hypot(event.clientX - pointer.x, event.clientY - pointer.y) < 6) return
    const rect = renderer.domElement.getBoundingClientRect()
    raycaster.current.setFromCamera(new THREE.Vector2(
      (event.clientX - rect.left) / rect.width * 2 - 1, 1 - (event.clientY - rect.top) / rect.height * 2,
    ), camera)
    const heading = pickGoalHeading(raycaster.current, pointer.goal)
    if (heading !== null) { pointer.heading = heading; onGoalHeadingChange?.(heading) }
  }

  const handleGoalHeadingPointerDown = (event: React.PointerEvent<HTMLDivElement>) => {
    if (!goalGestureEnabled
      || event.button !== 0 || event.shiftKey || event.altKey || goalHeadingPointerRef.current) return
    event.preventDefault(); event.stopPropagation(); mouseDownPos.current = null
    const adjusting = goalHeadingEditing && pendingGoal
    let goal = adjusting ? pendingGoal : null
    if (!goal) {
      const point = navigationPointAt(event.clientX, event.clientY, goalPickMode)
      if (!point) return
      const [x, y, z] = threeToLingtu([point.x, point.y, point.z])
      goal = onPendingGoal?.(x, y, z, goalPickMode) || null
    }
    if (!goal || !Number.isFinite(goal.z)) return
    goalHeadingPointerRef.current = { pointerId: event.pointerId, yaw: adjusting ? pendingGoalYaw : undefined,
      heading: undefined, goal, x: event.clientX, y: event.clientY,
      element: event.currentTarget, controlsEnabled: controlsRef.current?.enabled ?? true }
    event.currentTarget.setPointerCapture(event.pointerId)
    if (controlsRef.current) controlsRef.current.enabled = false
  }

  const finishGoalHeading = (event: React.PointerEvent<HTMLDivElement>, cancelled: boolean) => {
    const pointer = goalHeadingPointerRef.current
    if (!pointer || pointer.pointerId !== event.pointerId) return
    event.preventDefault(); event.stopPropagation()
    if (!cancelled) handleGoalHeadingPointerMove(event)
    else onGoalHeadingChange?.(pointer.yaw)
    mouseDownPos.current = null
    clearGoalHeadingPointer()
    if ((cancelled ? pointer.yaw : pointer.heading) !== undefined) onGoalHeadingComplete?.()
  }

  // ── Click vs drag detection ────────────────────────────────────
  const mouseDownPos = useRef<{ x: number; y: number; shift: boolean; alt: boolean } | null>(null)

  const handleMouseDown = (e: React.MouseEvent<HTMLDivElement>) => {
    if (stairTool || mapEdit || e.button !== 0 || (goalGestureEnabled && !e.shiftKey && !e.altKey)) return
    mouseDownPos.current = { x: e.clientX, y: e.clientY, shift: e.shiftKey, alt: e.altKey }
  }

  const handleMouseUp = (e: React.MouseEvent<HTMLDivElement>) => {
    if (stairTool || mapEdit) return
    if (!onPendingGoal && !onRelocalize) return
    const down = mouseDownPos.current
    mouseDownPos.current = null
    if (!down) return
    const dx = e.clientX - down.x
    const dy = e.clientY - down.y
    // Only treat as click if mouse moved < 5px (not a drag/orbit)
    if (Math.hypot(dx, dy) >= 5) return

    // Ordinary navigation clicks use the current body-height plane. Selecting
    // a 3D surface is explicit, so ceiling/wall returns cannot change it by accident.
    const shiftDown = down.shift || e.shiftKey
    const mode = mappingObservationVisible ? 'plane'
      : shiftDown || down.alt || e.altKey ? 'surface' : goalPickMode
    const p = navigationPointAt(e.clientX, e.clientY, mode)
    if (p) {
      const [wx, wy, wz] = threeToLingtu([p.x, p.y, p.z])
      // Shift+click → relocalize (set initial pose); plain click → goal
      if (shiftDown && onRelocalize) {
        onRelocalize(wx, wy)
      } else {
        onPendingGoal?.(wx, wy, wz, mode)
      }
    }
  }

  return (
    <div
      ref={mountRef}
      onPointerDownCapture={event => {
        const handlePointer = mapEdit ? handleMapEditPointerDown : stairTool ? handleStairPointerDown : handleGoalHeadingPointerDown
        handlePointer(event)
      }}
      onPointerMoveCapture={mapEdit ? handleMapEditPointerMove : stairTool ? handleStairPointerMove : handleGoalHeadingPointerMove}
      onPointerUpCapture={mapEdit ? handleMapEditPointerUp : stairTool ? handleStairPointerUp : event => finishGoalHeading(event, false)}
      onPointerCancel={mapEdit ? clearMapEditPointer : stairTool ? clearStairPointer : event => finishGoalHeading(event, true)}
      onMouseDown={handleMouseDown}
      onMouseUp={handleMouseUp}
      style={{ width: '100%', height: '100%', cursor: stairTool || mapEdit?.selecting || (!mapEdit && onPendingGoal) ? 'crosshair' : 'grab', position: 'relative', touchAction: stairTool || mapEdit?.selecting || goalGestureEnabled ? 'none' : undefined }}
    >
      <div ref={stairSelectionRef} aria-hidden="true" style={{ display: 'none', position: 'absolute', pointerEvents: 'none',
        border: '1px solid #78b8df', background: 'rgba(120,184,223,0.12)', zIndex: 3 }} />
      <div ref={robotLoadStatusRef} role="status"
        style={{ position: 'absolute', bottom: 12, left: 12, zIndex: 2, pointerEvents: 'none',
          color: 'var(--text)', background: 'var(--bg-layer-1)', borderRadius: 6, fontSize: 12 }} />
    </div>
  )
})
