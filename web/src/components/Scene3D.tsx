/**
 * Scene3D — Three.js 3D map visualization
 *
 * Coordinate mapping:
 *   World (LingTu): X right, Y forward, Z up
 *   Three.js:       X right, Y up,      Z toward camera
 *   → Three.js pos = (worldX, worldZ_height, -worldY)
 */
import { useRef, useEffect, forwardRef, useImperativeHandle } from 'react'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import type { PathPoint, CostmapEvent, SlopeGridEvent, SceneGraphEvent, MapSceneEvent, NavigationDdsSnapshotResponse } from '../types'
import type { BinaryCloud } from '../hooks/useBinaryCloud'
import {
  cloudFrameSharesSavedMapFrame,
  cloudFramesShareCoordinateEpoch,
} from '../workers/cloudDecoderCore.ts'
import { createCostmapLayer } from './scene3d/layers/costmapLayer'
import { disposeLiveCloudLayer, upsertLiveCloudLayer } from './scene3d/layers/liveCloudLayer'
import {
  createSavedMapLayer,
  SAVED_MAP_Z_CEIL,
  SAVED_MAP_Z_FLOOR,
  updateSavedMapPointSize,
} from './scene3d/layers/savedMapLayer'
import { createSlopeLayer } from './scene3d/layers/slopeLayer'
import { disposeGroupedMesh, type GroupedMesh } from './scene3d/layers/layerUtils'
import { createThunderV4Model } from './scene3d/robot/thunderV4Model'
import {
  createLocalPlannerDiagnosticLayer,
  disposeLocalPlannerDiagnosticLayer,
} from './scene3d/layers/localPlannerLayer'

export interface Scene3DHandle {
  resetCamera(): void
}

interface Layers {
  grid:    boolean
  cloud:   boolean
  trail:   boolean
  path:    boolean
  goal:    boolean
  robot:   boolean
  costmap: boolean
  slope:   boolean
  localPlanner: boolean
}

interface Scene3DProps {
  cloud:        BinaryCloud
  scanCloud?:   BinaryCloud | null
  savedMapFlat?: number[]
  savedMapFrameId?: string | null
  savedMapEpoch?: number | null
  mapScene?:    MapSceneEvent | null
  costmap:      CostmapEvent | null
  slopeGrid:    SlopeGridEvent | null
  sceneGraph:   SceneGraphEvent | null
  robotX:       number
  robotY:       number
  robotValid:   boolean
  yaw:          number
  trail:        Array<[number, number]>
  path:         PathPoint[]
  localPath:    PathPoint[]
  localPlannerSnapshot?: NavigationDdsSnapshotResponse | null
  layers:       Layers
  pointSize:    number
  onPendingGoal: (x: number, y: number) => void
  onRelocalize?: (x: number, y: number) => void
  pendingGoal?:  { x: number; y: number } | null
}

const LIVE_SCAN_COLOR = 0x68f7e1
// The worker owns binary decode, filtering, color mapping, and coordinate
// conversion. Scene3D consumes those typed arrays as a true 3D Points layer.

function removeFrom(scene: THREE.Scene, obj: THREE.Object3D | undefined | null) {
  if (!obj) return
  scene.remove(obj)
  if ((obj as THREE.Mesh).geometry) (obj as THREE.Mesh).geometry.dispose()
  const mat = (obj as THREE.Mesh).material
  if (Array.isArray(mat)) mat.forEach(m => m.dispose())
  else if (mat) mat.dispose()
}

export const Scene3D = forwardRef<Scene3DHandle, Scene3DProps>(function Scene3D(
  { cloud, scanCloud, savedMapFlat, savedMapFrameId, savedMapEpoch, costmap, slopeGrid, sceneGraph, robotX, robotY, robotValid, yaw, trail, path, localPath, localPlannerSnapshot, layers, pointSize, onPendingGoal, onRelocalize, pendingGoal },
  ref,
) {
  const mountRef   = useRef<HTMLDivElement>(null)
  const sceneRef   = useRef<THREE.Scene | null>(null)
  const cameraRef  = useRef<THREE.PerspectiveCamera | null>(null)
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null)
  const controlsRef = useRef<OrbitControls | null>(null)
  const rafRef     = useRef(0)
  const scanFrameAligned = scanCloud != null
    && cloudFramesShareCoordinateEpoch(cloud, scanCloud)
  const savedMapFrameAligned = cloudFrameSharesSavedMapFrame(
    cloud,
    savedMapFrameId,
    savedMapEpoch,
  )
  const hasSavedMap = savedMapFlat !== undefined && savedMapFlat.length >= 3
  const liveCloudFrameAllowed = !hasSavedMap || savedMapFrameAligned

  // Scene objects — recreated on data change
  const trailLineRef = useRef<THREE.Line | null>(null)
  const pathLineRef  = useRef<THREE.Mesh | null>(null)
  const localPathRef = useRef<THREE.Mesh | null>(null)
  const robotRef   = useRef<THREE.Group | null>(null)
  const goalRef        = useRef<THREE.Mesh | null>(null)
  const pendingGoalRef = useRef<THREE.Mesh | null>(null)
  const costmapMeshRef = useRef<GroupedMesh | null>(null)
  const slopeMeshRef   = useRef<GroupedMesh | null>(null)
  const localPlannerRef = useRef<THREE.Group | null>(null)
  const gridRef        = useRef<THREE.GridHelper | null>(null)
  const floorRef   = useRef<THREE.Mesh | null>(null)
  const liveCloudRef   = useRef<THREE.Points | null>(null)
  const scanCloudRef   = useRef<THREE.Points | null>(null)
  const savedMapRef    = useRef<THREE.Points | null>(null)
  const pointSizeRef   = useRef(pointSize)
  const sgGroupRef     = useRef<THREE.Group | null>(null)
  const raycaster  = useRef(new THREE.Raycaster())
  const robotPosRef = useRef({ x: 0, y: 0 })

  // ── Expose resetCamera ──────────────────────────────────────────
  useImperativeHandle(ref, () => ({
    resetCamera() {
      const { x, y } = robotPosRef.current
      cameraRef.current?.position.set(x, 20, -y + 18)
      if (controlsRef.current) {
        controlsRef.current.target.set(x, 0, -y)
        controlsRef.current.update()
      }
    },
  }), [])

  // ── Init (once) ─────────────────────────────────────────────────
  useEffect(() => {
    const mount = mountRef.current!
    const w = mount.clientWidth, h = mount.clientHeight

    const scene = new THREE.Scene()
    scene.background = new THREE.Color(0x07070e)
    sceneRef.current = scene

    const camera = new THREE.PerspectiveCamera(50, w / h, 0.1, 500)
    camera.position.set(0, 20, 18)
    camera.lookAt(0, 0, 0)
    cameraRef.current = camera

    const renderer = new THREE.WebGLRenderer({ antialias: true })
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
    renderer.setSize(w, h)
    mount.appendChild(renderer.domElement)
    rendererRef.current = renderer

    const controls = new OrbitControls(camera, renderer.domElement)
    controls.enableDamping  = true
    controls.dampingFactor  = 0.1
    controls.minDistance    = 2
    controls.maxDistance    = 120
    controls.maxPolarAngle  = Math.PI * 0.475
    controlsRef.current = controls

    scene.add(new THREE.AmbientLight(0xffffff, 0.55))
    const dir = new THREE.DirectionalLight(0x7799ff, 0.9)
    dir.position.set(10, 25, 8)
    scene.add(dir)

    const grid = new THREE.GridHelper(300, 150, 0x18182e, 0x111126)
    grid.position.y = -0.02
    scene.add(grid)
    gridRef.current = grid

    // Invisible floor plane for click raycasting
    const floor = new THREE.Mesh(
      new THREE.PlaneGeometry(600, 600),
      new THREE.MeshBasicMaterial({ visible: false, side: THREE.DoubleSide }),
    )
    floor.rotation.x = -Math.PI / 2
    scene.add(floor)
    floorRef.current = floor

    const animate = () => {
      rafRef.current = requestAnimationFrame(animate)
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
      ro.disconnect()
      controls.dispose()
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
      if (scanCloudRef.current) {
        disposeLiveCloudLayer(scene, scanCloudRef.current)
        scanCloudRef.current = null
      }
      return
    }

    liveCloudRef.current = upsertLiveCloudLayer(scene, liveCloudRef.current, cloud, pointSize, {
      opacity: 0.64,
      pointSizeScale: 0.9,
      renderOrder: 6,
      vertexColors: true,
    })
  }, [cloud, layers.cloud, liveCloudFrameAllowed, pointSize])

  // Current scan overlay.  This is intentionally separate from the accumulated
  // map cloud so mapping mode can be stable and still show live sensor motion.
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    if (!layers.cloud || !liveCloudFrameAllowed || !scanCloud || !scanFrameAligned) {
      if (scanCloudRef.current) scanCloudRef.current.visible = false
      return
    }
    scanCloudRef.current = upsertLiveCloudLayer(scene, scanCloudRef.current, scanCloud, pointSize, {
      color: LIVE_SCAN_COLOR,
      opacity: 0.92,
      pointSizeScale: 1.18,
      renderOrder: 12,
      vertexColors: false,
    })
  }, [scanCloud, scanFrameAligned, layers.cloud, liveCloudFrameAllowed, pointSize])

  // ── Trail ───────────────────────────────────────────────────────
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    if (trailLineRef.current) { removeFrom(scene, trailLineRef.current); trailLineRef.current = null }
    if (!layers.trail || trail.length < 2) return

    const pts = trail.map(([x, y]) => new THREE.Vector3(x, 0.25, -y))
    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(pts),
      new THREE.LineBasicMaterial({ color: 0xff9f43, transparent: true, opacity: 0.95 }),
    )
    scene.add(line)
    trailLineRef.current = line
  }, [trail, layers.trail])

  // ── Global path — purple TubeGeometry (slightly thinner than local path).
  // 管子比局部路径细一档,维持视觉层次: 全局粗略 + 局部精细。
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    if (pathLineRef.current) { removeFrom(scene, pathLineRef.current); pathLineRef.current = null }
    if (!layers.path || path.length < 2) return

    const rawPts = path.map(p => new THREE.Vector3(p.x, 0.18, -p.y))
    const curve = new THREE.CatmullRomCurve3(rawPts, false, 'catmullrom', 0.5)
    const tubularSegments = Math.max(rawPts.length * 6, 60)
    const geo = new THREE.TubeGeometry(curve, tubularSegments, 0.04, 6, false)
    const mat = new THREE.MeshBasicMaterial({ color: 0xa855f7 })  // purple
    const mesh = new THREE.Mesh(geo, mat)
    scene.add(mesh)
    pathLineRef.current = mesh
  }, [path, layers.path])

  // ── Local path — neon-green TubeGeometry.
  // 用 Tube 而不是 Line,因为 WebGL 的 LineBasicMaterial.linewidth 在 Chrome/Edge
  // 被 ANGLE 硬限制为 1px,1.5m 的避障路径在屏幕上就是一根发丝,肉眼看不见。
  // z=0.28 抬到紫色全局 (z=0.10) 和底图点云之上,不会被盖。
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    if (localPathRef.current) { removeFrom(scene, localPathRef.current); localPathRef.current = null }
    if (!layers.path || localPath.length < 2) return

    const rawPts = localPath.map(p => new THREE.Vector3(p.x, 0.28, -p.y))
    const curve = new THREE.CatmullRomCurve3(rawPts, false, 'catmullrom', 0.5)
    const tubularSegments = Math.max(rawPts.length * 4, 40)
    const geo = new THREE.TubeGeometry(curve, tubularSegments, 0.06, 6, false)
    const mat = new THREE.MeshBasicMaterial({ color: 0x00ffa3 })  // neon green
    const mesh = new THREE.Mesh(geo, mat)
    scene.add(mesh)
    localPathRef.current = mesh
  }, [localPath, layers.path])

  // ── Robot model ─────────────────────────────────────────────────
  useEffect(() => {
    if (robotValid) robotPosRef.current = { x: robotX, y: robotY }
    const scene = sceneRef.current
    if (!scene) return

    if (!robotRef.current) {
      const g = createThunderV4Model()
      scene.add(g)
      robotRef.current = g
    }

    robotRef.current.visible = layers.robot && robotValid
    if (layers.robot && robotValid) {
      robotRef.current.position.set(robotX, 0, -robotY)
      robotRef.current.rotation.y = yaw
    }
  }, [robotX, robotY, robotValid, yaw, layers.robot])

  // ── Goal marker ─────────────────────────────────────────────────
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    if (goalRef.current) { removeFrom(scene, goalRef.current); goalRef.current = null }
    if (!layers.goal || path.length === 0) return

    const last = path[path.length - 1]
    const mesh = new THREE.Mesh(
      new THREE.SphereGeometry(0.3, 8, 8),
      new THREE.MeshBasicMaterial({ color: 0xfbbf24, wireframe: true }),
    )
    mesh.position.set(last.x, 0.3, -last.y)
    scene.add(mesh)
    goalRef.current = mesh
  }, [path, layers.goal])

  // ── Pending goal marker ────────────────────────────────────────
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    if (pendingGoalRef.current) { removeFrom(scene, pendingGoalRef.current); pendingGoalRef.current = null }
    if (!pendingGoal) return

    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(0.4, 0.06, 8, 32),
      new THREE.MeshBasicMaterial({ color: 0x06b6d4 }),
    )
    ring.rotation.x = Math.PI / 2
    ring.position.set(pendingGoal.x, 0.05, -pendingGoal.y)
    scene.add(ring)
    pendingGoalRef.current = ring
  }, [pendingGoal])

  // Costmap overlay
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return

    disposeGroupedMesh(scene, costmapMeshRef.current)
    costmapMeshRef.current = null

    if (!costmap || !layers.costmap) return

    const mesh = createCostmapLayer(costmap)
    if (!mesh) return
    scene.add(mesh._group ?? mesh)
    costmapMeshRef.current = mesh
  }, [costmap, layers.costmap])


  // ── Costmap visibility toggle ──────────────────────────────────
  useEffect(() => {
    if (costmapMeshRef.current) costmapMeshRef.current.visible = layers.costmap
  }, [layers.costmap])

  // Slope grid overlay
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return

    disposeGroupedMesh(scene, slopeMeshRef.current)
    slopeMeshRef.current = null

    if (!slopeGrid || !layers.slope) return

    const mesh = createSlopeLayer(slopeGrid)
    if (!mesh) return
    scene.add(mesh._group ?? mesh)
    slopeMeshRef.current = mesh
  }, [slopeGrid, layers.slope])

  // Native endpoint diagnostics are read-only and have no path back into
  // planning or control. Rebuild only at the explicitly enabled low poll rate.
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return

    disposeLocalPlannerDiagnosticLayer(scene, localPlannerRef.current)
    localPlannerRef.current = null
    if (!layers.localPlanner) return

    const group = createLocalPlannerDiagnosticLayer(localPlannerSnapshot)
    if (!group) return
    scene.add(group)
    localPlannerRef.current = group

    return () => {
      if (localPlannerRef.current === group) {
        disposeLocalPlannerDiagnosticLayer(scene, group)
        localPlannerRef.current = null
      }
    }
  }, [localPlannerSnapshot, layers.localPlanner])


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
      const tx = obj.x, tz = -obj.y, ty = 0.5
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
    const hits = raycaster.current.intersectObject(floor)
    if (hits.length > 0) {
      const p = hits[0].point
      const wx = p.x
      const wy = -p.z  // convert Three.js back to world coords
      // Shift+click → relocalize (set initial pose); plain click → goal
      const shiftDown = (down.shift || e.shiftKey)
      if (shiftDown && onRelocalize) {
        onRelocalize(wx, wy)
      } else {
        onPendingGoal(wx, wy)
      }
    }
  }

  return (
    <div
      ref={mountRef}
      onMouseDown={handleMouseDown}
      onMouseUp={handleMouseUp}
      style={{ width: '100%', height: '100%', cursor: 'crosshair', position: 'relative' }}
    />
  )
})
