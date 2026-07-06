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
import type { PathPoint, CostmapEvent, SlopeGridEvent, SceneGraphEvent } from '../types'
import type { BinaryCloud } from '../hooks/useBinaryCloud'

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
}

interface Scene3DProps {
  cloud:        BinaryCloud
  savedMapFlat?: number[]
  costmap:      CostmapEvent | null
  slopeGrid:    SlopeGridEvent | null
  sceneGraph:   SceneGraphEvent | null
  robotX:       number
  robotY:       number
  yaw:          number
  trail:        Array<[number, number]>
  path:         PathPoint[]
  localPath:    PathPoint[]
  layers:       Layers
  pointSize:    number
  onPendingGoal: (x: number, y: number) => void
  onRelocalize?: (x: number, y: number) => void
  pendingGoal?:  { x: number; y: number } | null
}

const Z_FLOOR   = -0.2  // include floor-level scan points
const Z_CEIL    = 2.8   // ignore points above ceiling (m)
// The worker still owns binary decode and filtering. Scene3D projects a sampled
// live cloud onto a 2D overlay so points remain visible across WebGL drivers.

function removeFrom(scene: THREE.Scene, obj: THREE.Object3D | undefined | null) {
  if (!obj) return
  scene.remove(obj)
  if ((obj as THREE.Mesh).geometry) (obj as THREE.Mesh).geometry.dispose()
  const mat = (obj as THREE.Mesh).material
  if (Array.isArray(mat)) mat.forEach(m => m.dispose())
  else if (mat) mat.dispose()
}

export const Scene3D = forwardRef<Scene3DHandle, Scene3DProps>(function Scene3D(
  { cloud, savedMapFlat, costmap, slopeGrid, sceneGraph, robotX, robotY, yaw, trail, path, localPath, layers, pointSize, onPendingGoal, onRelocalize, pendingGoal },
  ref,
) {
  const mountRef   = useRef<HTMLDivElement>(null)
  const sceneRef   = useRef<THREE.Scene | null>(null)
  const cameraRef  = useRef<THREE.PerspectiveCamera | null>(null)
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null)
  const cloudOverlayRef = useRef<HTMLCanvasElement | null>(null)
  const controlsRef = useRef<OrbitControls | null>(null)
  const rafRef     = useRef(0)
  const latestCloudRef = useRef(cloud)
  const cloudVisibleRef = useRef(layers.cloud)
  const pointSizeRef = useRef(pointSize)

  // Scene objects — recreated on data change
  const trailLineRef = useRef<THREE.Line | null>(null)
  const pathLineRef  = useRef<THREE.Mesh | null>(null)
  const localPathRef = useRef<THREE.Mesh | null>(null)
  const robotRef   = useRef<THREE.Group | null>(null)
  const goalRef        = useRef<THREE.Mesh | null>(null)
  const pendingGoalRef = useRef<THREE.Mesh | null>(null)
  const costmapMeshRef = useRef<THREE.Mesh | null>(null)
  const slopeMeshRef   = useRef<THREE.Mesh | null>(null)
  const gridRef        = useRef<THREE.GridHelper | null>(null)
  const floorRef   = useRef<THREE.Mesh | null>(null)
  const savedMapRef    = useRef<THREE.Points | null>(null)
  const sgGroupRef     = useRef<THREE.Group | null>(null)
  const raycaster  = useRef(new THREE.Raycaster())
  const robotPosRef = useRef({ x: 0, y: 0 })

  useEffect(() => { latestCloudRef.current = cloud }, [cloud])
  useEffect(() => { cloudVisibleRef.current = layers.cloud }, [layers.cloud])
  useEffect(() => { pointSizeRef.current = pointSize }, [pointSize])

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

    const cloudOverlay = document.createElement('canvas')
    cloudOverlay.style.position = 'absolute'
    cloudOverlay.style.inset = '0'
    cloudOverlay.style.width = '100%'
    cloudOverlay.style.height = '100%'
    cloudOverlay.style.pointerEvents = 'none'
    cloudOverlay.style.zIndex = '1'
    mount.appendChild(cloudOverlay)
    cloudOverlayRef.current = cloudOverlay

    const resizeOverlay = (nw: number, nh: number) => {
      const dpr = Math.min(window.devicePixelRatio || 1, 2)
      cloudOverlay.width = Math.max(1, Math.floor(nw * dpr))
      cloudOverlay.height = Math.max(1, Math.floor(nh * dpr))
    }
    resizeOverlay(w, h)

    const drawCloudOverlay = () => {
      const overlay = cloudOverlayRef.current
      if (!overlay) return
      const ctx = overlay.getContext('2d')
      if (!ctx) return
      const width = overlay.width
      const height = overlay.height
      ctx.clearRect(0, 0, width, height)
      const live = latestCloudRef.current
      if (!cloudVisibleRef.current || live.count <= 0) return
      const stride = Math.max(1, Math.ceil(live.count / 2500))
      const dpr = Math.min(window.devicePixelRatio || 1, 2)
      const size = Math.max(2, pointSizeRef.current * 10) * dpr
      const half = size / 2
      const p = new THREE.Vector3()
      ctx.fillStyle = 'rgba(94, 234, 255, 0.88)'
      for (let src = 0; src < live.count; src += stride) {
        const off = src * 3
        p.set(live.positions[off], live.positions[off + 1], live.positions[off + 2])
        p.project(camera)
        if (p.z < -1 || p.z > 1) continue
        const sx = (p.x * 0.5 + 0.5) * width
        const sy = (-p.y * 0.5 + 0.5) * height
        if (sx < -size || sx > width + size || sy < -size || sy > height + size) continue
        ctx.fillRect(sx - half, sy - half, size, size)
      }
    }

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
      drawCloudOverlay()
    }
    animate()

    const ro = new ResizeObserver(() => {
      const nw = mount.clientWidth, nh = mount.clientHeight
      camera.aspect = nw / nh
      camera.updateProjectionMatrix()
      renderer.setSize(nw, nh)
      resizeOverlay(nw, nh)
    })
    ro.observe(mount)

    return () => {
      cancelAnimationFrame(rafRef.current)
      ro.disconnect()
      controls.dispose()
      renderer.dispose()
      if (mount.contains(renderer.domElement)) mount.removeChild(renderer.domElement)
      if (cloudOverlayRef.current && mount.contains(cloudOverlayRef.current)) {
        mount.removeChild(cloudOverlayRef.current)
      }
      cloudOverlayRef.current = null
    }
  }, [])

  // ── Grid visibility ─────────────────────────────────────────────
  useEffect(() => {
    if (gridRef.current) gridRef.current.visible = layers.grid
  }, [layers.grid])

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
    robotPosRef.current = { x: robotX, y: robotY }
    const scene = sceneRef.current
    if (!scene) return

    if (!robotRef.current) {
      const g = new THREE.Group()

      // Body wireframe box
      g.add(new THREE.LineSegments(
        new THREE.EdgesGeometry(new THREE.BoxGeometry(0.68, 0.22, 0.34)),
        new THREE.LineBasicMaterial({ color: 0xf472b6 }),
      ))

      // Heading arrow
      g.add(new THREE.ArrowHelper(
        new THREE.Vector3(1, 0, 0),
        new THREE.Vector3(0.3, 0.11, 0),
        0.55, 0xffffff, 0.18, 0.1,
      ))

      // Four leg dots (foot contact points)
      const legGeo = new THREE.SphereGeometry(0.04, 5, 5)
      const legMat = new THREE.MeshBasicMaterial({ color: 0xe2e8f0 });
      ([
        [0.28, -0.11,  0.18],
        [-0.28, -0.11, 0.18],
        [0.28, -0.11, -0.18],
        [-0.28, -0.11, -0.18],
      ] as [number, number, number][]).forEach(([x, y, z]) => {
        const m = new THREE.Mesh(legGeo, legMat)
        m.position.set(x, y, z)
        g.add(m)
      })

      // Camera frustum (sky-blue wireframe pyramid)
      const fp = [
        new THREE.Vector3(0.35, 0.11, 0),     // apex
        new THREE.Vector3(0.9,  0.48, 0.38),  // TL
        new THREE.Vector3(0.9,  0.48, -0.38), // TR
        new THREE.Vector3(0.9, -0.08, 0.38),  // BL
        new THREE.Vector3(0.9, -0.08, -0.38), // BR
      ]
      const edges = [[0,1],[0,2],[0,3],[0,4],[1,2],[2,4],[4,3],[3,1]]
      const fPos: number[] = []
      edges.forEach(([a, b]) => {
        fPos.push(fp[a].x, fp[a].y, fp[a].z, fp[b].x, fp[b].y, fp[b].z)
      })
      const fGeo = new THREE.BufferGeometry()
      fGeo.setAttribute('position', new THREE.Float32BufferAttribute(fPos, 3))
      g.add(new THREE.LineSegments(fGeo, new THREE.LineBasicMaterial({
        color: 0x38bdf8, transparent: true, opacity: 0.55,
      })))

      scene.add(g)
      robotRef.current = g
    }

    robotRef.current.visible = layers.robot
    if (layers.robot) {
      robotRef.current.position.set(robotX, 0, -robotY)
      robotRef.current.rotation.y = yaw
    }
  }, [robotX, robotY, yaw, layers.robot])

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

  // ── Costmap overlay ────────────────────────────────────────────
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return

    if (costmapMeshRef.current) {
      const m = costmapMeshRef.current as THREE.Mesh & { _group?: THREE.Group }
      const parent = m._group ?? m
      scene.remove(parent)
      m.geometry.dispose()
      ;(m.material as THREE.MeshBasicMaterial).map?.dispose()
      ;(m.material as THREE.Material).dispose()
      costmapMeshRef.current = null
    }

    if (!costmap || !layers.costmap) return

    const { grid_b64, cols, resolution, origin } = costmap
    const bytes = Uint8Array.from(atob(grid_b64), c => c.charCodeAt(0))
    // Prefer explicit rows from backend; fall back to byte-length inference
    // for older events that only carry cols.
    const rows  = costmap.rows ?? Math.round(bytes.length / cols)
    if (rows <= 0 || cols <= 0) return

    // Draw costmap to an offscreen canvas
    const canvas = document.createElement('canvas')
    canvas.width  = cols
    canvas.height = rows
    const ctx = canvas.getContext('2d')!
    const img = ctx.createImageData(cols, rows)

    // VGSwarm (ICRA 2024) style continuous gradient: 5-stop palette on
    // cost ∈ [0,100]. Stops are in sRGB — smooth in perceptual space
    // rather than HSL to avoid rainbow banding.
    //   0   fully transparent (free)
    //   20  cool teal     rgb( 46,196,182)  — traversable margin
    //   40  warm yellow   rgb(255,214, 90)  — soft edge
    //   60  saturated orange rgb(255,133, 38) — danger ring
    //   80  deep red      rgb(231, 72, 72)   — imminent collision
    //   100 lethal crimson rgb(183, 28, 28)  — wall / lethal
    // Keep alpha modest so this diagnostic layer does not bury live cloud points.
    const STOPS: Array<[number, number, number, number, number]> = [
      [  0,  46, 196, 182,   0],
      [ 20,  46, 196, 182,  44],
      [ 40, 255, 214,  90,  74],
      [ 60, 255, 133,  38, 108],
      [ 80, 231,  72,  72, 138],
      [100, 183,  28,  28, 162],
    ]
    const interp = (v: number): [number, number, number, number] => {
      for (let i = 0; i < STOPS.length - 1; i++) {
        const [a, ar, ag, ab, aa] = STOPS[i]
        const [b, br, bg, bb, ba] = STOPS[i + 1]
        if (v <= b) {
          const t = (v - a) / Math.max(1e-6, b - a)
          return [
            Math.round(ar + (br - ar) * t),
            Math.round(ag + (bg - ag) * t),
            Math.round(ab + (bb - ab) * t),
            Math.round(aa + (ba - aa) * t),
          ]
        }
      }
      return [STOPS[STOPS.length - 1][1], STOPS[STOPS.length - 1][2],
              STOPS[STOPS.length - 1][3], STOPS[STOPS.length - 1][4]]
    }
    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < cols; c++) {
        const val  = bytes[r * cols + c]
        const o    = (r * cols + c) * 4
        if (val === 0) {
          img.data[o] = img.data[o+1] = img.data[o+2] = img.data[o+3] = 0
          continue
        }
        const [R, G, B, A] = interp(val)
        img.data[o]     = R
        img.data[o + 1] = G
        img.data[o + 2] = B
        img.data[o + 3] = A
      }
    }
    ctx.putImageData(img, 0, 0)

    const tex = new THREE.CanvasTexture(canvas)
    tex.flipY = false  // grid[iy,ix] row0=Y_min — no flip needed
    tex.minFilter = THREE.LinearFilter
    tex.magFilter = THREE.NearestFilter

    const sizeX = cols * resolution
    const sizeY = rows * resolution
    const geo   = new THREE.PlaneGeometry(sizeX, sizeY)
    const mat   = new THREE.MeshBasicMaterial({
      map: tex, transparent: true, depthWrite: false,
    })
    // Grid origin = bottom-left corner in *map* world coords (backend has
    // already composed T_map_odom translation). `yaw` is the map→odom yaw
    // in map frame (CCW around world Z). We rotate the center offset in
    // world XY to get the true mesh center. Three.js uses (worldX, Z_up,
    // -worldY), so positive world-yaw maps to NEGATIVE rotation around
    // Three's up axis — that's the symptom森哥 saw (mirror flip).
    const yaw = (costmap as { yaw?: number }).yaw ?? 0
    const hx = sizeX / 2, hy = sizeY / 2
    const cosY = Math.cos(yaw), sinY = Math.sin(yaw)
    const cx = origin[0] + cosY * hx - sinY * hy
    const cy = origin[1] + sinY * hx + cosY * hy
    // Two-layer transform so the yaw stays in world space and doesn't get
    // entangled with the plane's -PI/2 X-rotation:
    //   Group: world-space yaw + translation (CCW yaw in map frame maps to
    //          negative rotation around Three's Y because we mirror world
    //          Y to -Z in scene coords).
    //   Mesh:  lies flat on world XZ (x-rotation only).
    const mesh = new THREE.Mesh(geo, mat)
    mesh.rotation.x = -Math.PI / 2
    mesh.renderOrder = 5
    const group = new THREE.Group()
    group.renderOrder = 5
    // Derivation: plane local (mx, my) after x-rotation → Three (mx, 0, -my).
    // World CCW yaw in map frame composes with the y-mirror into +yaw around
    // Three Y. Don't negate.
    group.rotation.y = yaw
    group.position.set(cx, 0.01, -cy)
    group.add(mesh)
    scene.add(group)
    costmapMeshRef.current = mesh
    ;(mesh as THREE.Mesh & { _group?: THREE.Group })._group = group
  }, [costmap, layers.costmap])

  // ── Costmap visibility toggle ──────────────────────────────────
  useEffect(() => {
    if (costmapMeshRef.current) costmapMeshRef.current.visible = layers.costmap
  }, [layers.costmap])

  // ── Slope grid overlay (green→yellow→red) ──────────────────────
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return

    if (slopeMeshRef.current) {
      const m = slopeMeshRef.current as THREE.Mesh & { _group?: THREE.Group }
      scene.remove(m._group ?? m)
      m.geometry.dispose()
      ;(m.material as THREE.MeshBasicMaterial).map?.dispose()
      ;(m.material as THREE.Material).dispose()
      slopeMeshRef.current = null
    }

    if (!slopeGrid || !layers.slope || !slopeGrid.grid_b64) return

    const { grid_b64, cols, resolution, origin } = slopeGrid
    const bytes = Uint8Array.from(atob(grid_b64), c => c.charCodeAt(0))
    const rows = Math.round(bytes.length / cols)
    if (rows <= 0 || cols <= 0) return

    const canvas = document.createElement('canvas')
    canvas.width = cols
    canvas.height = rows
    const ctx = canvas.getContext('2d')!
    const img = ctx.createImageData(cols, rows)

    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < cols; c++) {
        const raw = bytes[r * cols + c]  // no manual flip — CanvasTexture.flipY handles it
        const deg = raw * (90.0 / 255.0)
        const o = (r * cols + c) * 4
        if (deg < 3) {
          // flat — fully transparent
          img.data[o] = img.data[o+1] = img.data[o+2] = img.data[o+3] = 0
        } else if (deg < 15) {
          // mild slope — green
          const t = (deg - 3) / 12
          img.data[o]     = Math.round(40 * t)
          img.data[o + 1] = Math.round(180 + 40 * t)
          img.data[o + 2] = Math.round(60 * t)
          img.data[o + 3] = Math.round(30 + 50 * t)
        } else if (deg < 25) {
          // moderate slope — yellow
          const t = (deg - 15) / 10
          img.data[o]     = Math.round(200 + 55 * t)
          img.data[o + 1] = Math.round(200 - 60 * t)
          img.data[o + 2] = 30
          img.data[o + 3] = Math.round(80 + 40 * t)
        } else {
          // steep slope — red
          img.data[o]     = 240
          img.data[o + 1] = 50
          img.data[o + 2] = 50
          img.data[o + 3] = 140
        }
      }
    }
    ctx.putImageData(img, 0, 0)

    const tex = new THREE.CanvasTexture(canvas)
    tex.flipY = false  // grid[iy,ix] row0=Y_min — no flip needed
    tex.minFilter = THREE.LinearFilter
    tex.magFilter = THREE.NearestFilter

    const sizeX = cols * resolution
    const sizeY = rows * resolution
    const geo = new THREE.PlaneGeometry(sizeX, sizeY)
    const mat = new THREE.MeshBasicMaterial({
      map: tex, transparent: true, depthWrite: false,
    })
    const sYaw = (slopeGrid as { yaw?: number }).yaw ?? 0
    const hxS = sizeX / 2, hyS = sizeY / 2
    const cosS = Math.cos(sYaw), sinS = Math.sin(sYaw)
    const cxS = origin[0] + cosS * hxS - sinS * hyS
    const cyS = origin[1] + sinS * hxS + cosS * hyS
    const mesh = new THREE.Mesh(geo, mat)
    mesh.rotation.x = -Math.PI / 2
    const groupS = new THREE.Group()
    groupS.rotation.y = sYaw
    groupS.position.set(cxS, 0.02, -cyS)
    groupS.add(mesh)
    ;(mesh as THREE.Mesh & { _group?: THREE.Group })._group = groupS
    scene.add(groupS)
    slopeMeshRef.current = mesh
  }, [slopeGrid, layers.slope])

  // ── Saved map cloud (gray, background layer) ───────────────────
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    if (savedMapRef.current) {
      scene.remove(savedMapRef.current)
      savedMapRef.current.geometry.dispose()
      ;(savedMapRef.current.material as THREE.Material).dispose()
      savedMapRef.current = null
    }
    if (!savedMapFlat || savedMapFlat.length < 3) return
    // 防御：底图点数特别大时，取步长采样避免 GPU 崩溃
    const MAX_SAVED_PTS = 80_000
    const totalTriples = Math.floor(savedMapFlat.length / 3)
    const stride = totalTriples > MAX_SAVED_PTS ? Math.ceil(totalTriples / MAX_SAVED_PTS) : 1
    const positions: number[] = []
    const step = stride * 3
    for (let i = 0; i + 2 < savedMapFlat.length; i += step) {
      const wx = savedMapFlat[i], wy = savedMapFlat[i + 1], wz = savedMapFlat[i + 2]
      if (wz < Z_FLOOR || wz > Z_CEIL) continue
      positions.push(wx, wz, -wy)
    }
    if (positions.length === 0) return
    const geo = new THREE.BufferGeometry()
    geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3))
    // 底图：暗蓝灰，作 reference；实时扫描会以暖色叠在上面
    const mat = new THREE.PointsMaterial({ size: 0.05, color: 0x334466, sizeAttenuation: true, opacity: 0.55, transparent: true })
    const pts = new THREE.Points(geo, mat)
    scene.add(pts)
    savedMapRef.current = pts
  }, [savedMapFlat])

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
