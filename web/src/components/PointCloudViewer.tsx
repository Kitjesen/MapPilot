import { useEffect, useId, useRef, useState, useCallback, type CSSProperties } from 'react'
import { ChevronDown, ChevronUp, Maximize2, RotateCcw, SlidersHorizontal, X } from 'lucide-react'
import styles from './PointCloudViewer.module.css'
import { parsePcd, pcdCameraDistance, pcdPointCount } from '../services/pcdPreview.ts'
import { pickVisiblePcdPoint, projectPcdPoint, type PointCloudPick } from '../services/pcdPicking.ts'
import { fetchMapPreview } from '../services/mapPreview.ts'
import { CLOUD_PALETTES, DEFAULT_CLOUD_STYLE, readCloudStyle, saveCloudStyle, cloudStyleColors, type CloudStyle } from '../services/cloudStyle.ts'
import type { RecordingCloudView } from '../types/recordingScene.ts'

export type { PointCloudPick } from '../services/pcdPicking.ts'

interface HeightColors { lo: number[]; hi: number[] }
type ViewMode = '2d' | '3d'

function heightColors(canvas: HTMLCanvasElement): HeightColors {
  const css = getComputedStyle(canvas)
  const color = (name: string) => css.getPropertyValue(name).split(',').map(value => Number(value) / 255)
  return { lo: color('--cloud-low'), hi: color('--cloud-high') }
}

function referenceGrid(x0: number, x1: number, y0: number, y1: number, z: number) {
  const interval = Math.max(x1 - x0, y1 - y0, 0.1) / 10
  const magnitude = 10 ** Math.floor(Math.log10(interval))
  const step = Math.ceil(interval / magnitude) * magnitude
  const points: number[] = []
  for (let x = Math.ceil(x0 / step) * step; x <= x1; x += step) points.push(x, y0, z, x, y1, z)
  for (let y = Math.ceil(y0 / step) * step; y <= y1; y += step) points.push(x0, y, z, x1, y, z)
  return { points: new Float32Array(points), step }
}

// ── WebGL shaders ─────────────────────────────────────────────
const VS = `
  attribute vec3 a_pos;
  uniform mat4 u_mvp;
  uniform float u_minZ;
  uniform float u_rangeZ;
  uniform float u_pointSize;
  varying float v_t;
  varying float v_z;
  void main() {
    // LingTu X-forward/Y-left/Z-up -> Three X/Y-up/Z (x,z,-y).
    gl_Position = u_mvp * vec4(a_pos.x, a_pos.z, -a_pos.y, 1.0);
    gl_PointSize = u_pointSize;
    v_t = clamp((a_pos.z - u_minZ) / max(u_rangeZ, 0.01), 0.0, 1.0);
    v_z = a_pos.z;
  }
`
const FS = `
  precision mediump float;
  varying float v_t;
  varying float v_z;
  uniform vec3 u_lo;
  uniform vec3 u_hi;
  uniform bool u_points;
  uniform float u_maxZ;
  uniform float u_opacity;
  void main() {
    if (u_points && v_z > u_maxZ) discard;
    if (u_points && distance(gl_PointCoord, vec2(0.5)) > 0.5) discard;
    gl_FragColor = u_points
      ? vec4(mix(u_lo, u_hi, v_t), u_opacity)
      : vec4(u_lo, u_opacity);
  }
`

function compileProg(gl: WebGLRenderingContext) {
  const shaders: WebGLShader[] = []
  const compile = (type: number, src: string) => {
    const s = gl.createShader(type)!
    shaders.push(s)
    gl.shaderSource(s, src); gl.compileShader(s)
    if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s) ?? '')
    return s
  }
  const p = gl.createProgram()!
  let linked = false
  try {
    gl.attachShader(p, compile(gl.VERTEX_SHADER, VS))
    gl.attachShader(p, compile(gl.FRAGMENT_SHADER, FS))
    gl.linkProgram(p)
    if (!gl.getProgramParameter(p, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(p) ?? '')
    linked = true
    return p
  } finally {
    shaders.forEach(shader => gl.deleteShader(shader))
    if (!linked) gl.deleteProgram(p)
  }
}

// ── Mat4 helpers (column-major) ────────────────────────────────
function perspective(fov: number, aspect: number, near: number, far: number) {
  const f = 1/Math.tan(fov/2), nf = 1/(near-far), m = new Float32Array(16)
  m[0]=f/aspect; m[5]=f; m[10]=(far+near)*nf; m[11]=-1; m[14]=2*far*near*nf
  return m
}
function orthographic(height: number, aspect: number, near: number, far: number) {
  const m = new Float32Array(16)
  m[0] = 1 / (height * aspect)
  m[5] = 1 / height
  m[10] = -2 / (far - near)
  m[14] = -(far + near) / (far - near)
  m[15] = 1
  return m
}
function lookAt(ex:number,ey:number,ez:number, tx:number,ty:number,tz:number) {
  const m = new Float32Array(16)
  let zx=ex-tx,zy=ey-ty,zz=ez-tz, l=Math.hypot(zx,zy,zz)+1e-10
  zx/=l; zy/=l; zz/=l
  let ux=0, uy=1; const uz=0; if (Math.abs(zy)>0.99) { ux=1; uy=0 }
  let xx=uy*zz-uz*zy, xy=uz*zx-ux*zz, xz=ux*zy-uy*zx
  l=Math.hypot(xx,xy,xz)+1e-10; xx/=l; xy/=l; xz/=l
  const yx=zy*xz-zz*xy, yy=zz*xx-zx*xz, yz=zx*xy-zy*xx
  m[0]=xx; m[4]=xy; m[8] =xz; m[12]=-(xx*ex+xy*ey+xz*ez)
  m[1]=yx; m[5]=yy; m[9] =yz; m[13]=-(yx*ex+yy*ey+yz*ez)
  m[2]=zx; m[6]=zy; m[10]=zz; m[14]=-(zx*ex+zy*ey+zz*ez)
  m[15]=1; return m
}
function mulM(a: Float32Array, b: Float32Array) {
  const c = new Float32Array(16)
  for (let i=0;i<4;i++) for (let j=0;j<4;j++) {
    let s=0; for (let k=0;k<4;k++) s+=a[i+k*4]*b[k+j*4]; c[i+j*4]=s
  }
  return c
}

function mvpFor(s: GLS) {
  const c = s.gl.canvas as HTMLCanvasElement
  const sinP=Math.sin(s.phi), cosP=Math.cos(s.phi)
  const eyeDistance = s.mode === '2d' ? Math.max(s.dist, s.radius * 2) : s.dist
  const ex=s.center[0]+eyeDistance*sinP*Math.cos(s.theta)
  const ey=s.center[1]+eyeDistance*cosP
  const ez=s.center[2]+eyeDistance*sinP*Math.sin(s.theta)
  return mulM(
    s.mode === '2d'
      ? orthographic(s.dist * Math.tan(Math.PI/8), c.width/c.height, 0.05, eyeDistance*20)
      : perspective(Math.PI/4, c.width/c.height, 0.05, eyeDistance*20),
    lookAt(ex,ey,ez, s.center[0],s.center[1],s.center[2]),
  )
}

// ── GL state ──────────────────────────────────────────────────
interface GLS {
  gl: WebGLRenderingContext; prog: WebGLProgram; vbuf: WebGLBuffer; nPts: number
  gridbuf: WebGLBuffer; gridCount: number
  pts: Float32Array
  minZ: number; rangeZ: number
  maxVisibleZ: number
  center: [number,number,number]; radius: number
  mapCenter: [number,number,number]
  theta: number; phi: number; dist: number
  fitted: boolean
  mode: ViewMode
  colors: HeightColors
  style: CloudStyle
  overlays?: Array<{ buffer: WebGLBuffer; count: number; kind: 'trajectory' | 'path' }>
}

function disposeCloudScene(scene: GLS) {
  scene.gl.deleteBuffer(scene.vbuf)
  scene.gl.deleteBuffer(scene.gridbuf)
  for (const overlay of scene.overlays ?? []) scene.gl.deleteBuffer(overlay.buffer)
  scene.gl.deleteProgram(scene.prog)
}

function uploadCloudScene(canvas: HTMLCanvasElement, pts: Float32Array, style: CloudStyle, previous?: GLS, lines: RecordingCloudView['lines'] = []) {
  let x0=Infinity,x1=-Infinity,y0=Infinity,y1=-Infinity,z0=Infinity,z1=-Infinity
  for (const positions of [pts, ...lines.map(line => line.points)]) for (let i = 0; i < positions.length; i += 3) {
    if (!Number.isFinite(positions[i]) || !Number.isFinite(positions[i + 1]) || !Number.isFinite(positions[i + 2])) continue
    x0 = Math.min(x0, positions[i]); x1 = Math.max(x1, positions[i])
    y0 = Math.min(y0, positions[i + 1]); y1 = Math.max(y1, positions[i + 1])
    z0 = Math.min(z0, positions[i + 2]); z1 = Math.max(z1, positions[i + 2])
  }
  if (!Number.isFinite(x0)) { x0 = y0 = z0 = -.5; x1 = y1 = z1 = .5 }
  const radius = Math.max(.1, Math.hypot(x1 - x0, y1 - y0, z1 - z0) / 2)
  const gl = previous?.gl ?? canvas.getContext('webgl')
  if (!gl) throw new Error('WebGL unavailable')
  gl.clearColor(0, 0, 0, 0); gl.enable(gl.DEPTH_TEST)
  const grid = referenceGrid(x0, x1, y0, y1, z0)
  const mapCenter: [number, number, number] = [(x0 + x1) / 2, (z0 + z1) / 2, -(y0 + y1) / 2]
  const scene: GLS = previous ?? {
    gl, prog: compileProg(gl), vbuf: gl.createBuffer()!, gridbuf: gl.createBuffer()!,
    nPts: 0, pts, gridCount: 0, minZ: z0, rangeZ: z1 - z0, maxVisibleZ: z1,
    center: [...mapCenter], mapCenter, radius, theta: -Math.PI / 2, phi: 0,
    dist: pcdCameraDistance(radius, canvas.width / canvas.height), fitted: true, mode: '2d',
    colors: heightColors(canvas), style,
  }
  const followCeiling = !previous || previous.maxVisibleZ >= previous.minZ + previous.rangeZ
  Object.assign(scene, { pts, nPts: pts.length / 3, minZ: z0, rangeZ: z1 - z0, gridCount: grid.points.length / 3, mapCenter, radius })
  if (followCeiling) scene.maxVisibleZ = z1
  gl.bindBuffer(gl.ARRAY_BUFFER, scene.gridbuf)
  gl.bufferData(gl.ARRAY_BUFFER, grid.points, gl.STATIC_DRAW)
  gl.bindBuffer(gl.ARRAY_BUFFER, scene.vbuf)
  gl.bufferData(gl.ARRAY_BUFFER, pts, gl.DYNAMIC_DRAW)
  for (const overlay of scene.overlays ?? []) gl.deleteBuffer(overlay.buffer)
  scene.overlays = lines.map(line => {
    const buffer = gl.createBuffer()!
    gl.bindBuffer(gl.ARRAY_BUFFER, buffer)
    gl.bufferData(gl.ARRAY_BUFFER, line.points, gl.DYNAMIC_DRAW)
    return { buffer, count: line.points.length / 3, kind: line.kind }
  })
  return { scene, width: x1 - x0, depth: y1 - y0, minZ: z0, maxZ: z1, gridStep: grid.step }
}

interface CloudInfo {
  count: number; visibleCount: number; originalCount: number | null
  width: number; depth: number; minZ: number; maxZ: number; gridStep: number
  contentEpoch: number; frameId: string
}

// ── Component ─────────────────────────────────────────────────
export function PointCloudViewer({
  mapName,
  contentEpoch,
  productSessionId,
  refreshToken = 0,
  pickedPoint,
  onPick,
  recording,
}: {
  mapName: string | null
  contentEpoch?: number | null
  productSessionId?: string | null
  refreshToken?: number
  pickedPoint?: PointCloudPick | null
  onPick?: (point: PointCloudPick | null) => void
  recording?: RecordingCloudView
}) {
  const canvasRef    = useRef<HTMLCanvasElement>(null)
  const containerRef = useRef<HTMLDivElement>(null)
  const glRef        = useRef<GLS | null>(null)
  const dragRef      = useRef<{ x: number; y: number; startX: number; startY: number; moved: boolean } | null>(null)
  const pickedRef    = useRef<PointCloudPick | null>(null)
  const onPickRef    = useRef(onPick)
  const settingsButtonRef = useRef<HTMLButtonElement>(null)
  const [appearance, setAppearance] = useState(readCloudStyle)
  const heightPaletteRef = useRef<CloudStyle['palette']>(appearance.palette === 'mono' ? 'blue' : appearance.palette)
  const appearanceRef = useRef(appearance)
  const [settingsOpen, setSettingsOpen] = useState(false)

  const [status,       setStatus      ] = useState<'idle'|'loading'|'done'|'error'>('idle')
  const [info,         setInfo        ] = useState<CloudInfo | null>(null)
  const [viewMode,     setViewMode    ] = useState<ViewMode>('2d')
  const [ceiling,      setCeiling     ] = useState<number | null>(null)
  const [errorMessage, setErrorMessage] = useState('')
  const [pickScreen,   setPickScreen  ] = useState<{ x: number; y: number } | null>(null)
  const canPick = onPick !== undefined
  const recordingMode = recording !== undefined
  const settingsId = useId()

  const updatePickScreen = useCallback((point = pickedRef.current) => {
    const s = glRef.current
    const canvas = canvasRef.current
    if (!s || !canvas || !point || s.style.opacity === 0 || point.z > s.maxVisibleZ || !onPickRef.current) {
      setPickScreen(null)
      return
    }
    const projected = projectPcdPoint(mvpFor(s), point, canvas.width, canvas.height)
    if (!projected) {
      setPickScreen(null)
      return
    }
    const rect = canvas.getBoundingClientRect()
    setPickScreen({
      x: projected.x * (rect.width / canvas.width),
      y: projected.y * (rect.height / canvas.height),
    })
  }, [])

  const draw = useCallback(() => {
    const s = glRef.current; if (!s) return
    const { gl, prog, vbuf, nPts, gridbuf, gridCount, minZ, rangeZ, colors } = s
    const c = gl.canvas as HTMLCanvasElement
    gl.viewport(0, 0, c.width, c.height)
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)
    const mvp = mvpFor(s)
    gl.useProgram(prog)
    gl.uniformMatrix4fv(gl.getUniformLocation(prog,'u_mvp'), false, mvp)
    gl.uniform1f(gl.getUniformLocation(prog,'u_minZ'), minZ)
    gl.uniform1f(gl.getUniformLocation(prog,'u_rangeZ'), rangeZ)
    gl.uniform1f(gl.getUniformLocation(prog,'u_maxZ'), s.maxVisibleZ)
    gl.uniform1f(gl.getUniformLocation(prog,'u_pointSize'), s.style.pointSize * (window.devicePixelRatio || 1))
    gl.uniform1f(gl.getUniformLocation(prog,'u_opacity'), .16)
    gl.uniform3fv(gl.getUniformLocation(prog,'u_lo'), colors.lo)
    gl.uniform3fv(gl.getUniformLocation(prog,'u_hi'), colors.hi)
    const loc = gl.getAttribLocation(prog,'a_pos')
    gl.enableVertexAttribArray(loc)
    gl.bindBuffer(gl.ARRAY_BUFFER, gridbuf)
    gl.vertexAttribPointer(loc, 3, gl.FLOAT, false, 0, 0)
    gl.uniform1i(gl.getUniformLocation(prog,'u_points'), 0)
    gl.enable(gl.BLEND)
    gl.blendFuncSeparate(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA, gl.ONE, gl.ONE_MINUS_SRC_ALPHA)
    gl.depthMask(false)
    if (s.style.grid) gl.drawArrays(gl.LINES, 0, gridCount)
    gl.depthMask(true)
    if (s.style.opacity === 1) gl.disable(gl.BLEND)
    gl.bindBuffer(gl.ARRAY_BUFFER, vbuf)
    gl.vertexAttribPointer(loc, 3, gl.FLOAT, false, 0, 0)
    gl.uniform1i(gl.getUniformLocation(prog,'u_points'), 1)
    gl.uniform1f(gl.getUniformLocation(prog,'u_opacity'), s.style.opacity)
    gl.drawArrays(gl.POINTS, 0, nPts)
    gl.uniform1i(gl.getUniformLocation(prog,'u_points'), 0)
    gl.uniform1f(gl.getUniformLocation(prog,'u_opacity'), 1)
    for (const overlay of s.overlays ?? []) {
      gl.uniform3fv(gl.getUniformLocation(prog,'u_lo'), overlay.kind === 'trajectory' ? [.16, .66, .77] : [.8, .6, .29])
      gl.bindBuffer(gl.ARRAY_BUFFER, overlay.buffer)
      gl.vertexAttribPointer(loc, 3, gl.FLOAT, false, 0, 0)
      gl.drawArrays(gl.LINES, 0, overlay.count)
    }
    updatePickScreen()
  }, [updatePickScreen])

  useEffect(() => {
    appearanceRef.current = appearance
    saveCloudStyle(appearance)
    const s = glRef.current
    const canvas = canvasRef.current
    if (!s || !canvas) return
    s.style = appearance
    s.colors = heightColors(canvas)
    draw()
  }, [appearance, draw])

  const resizeCanvas = useCallback(() => {
    const canvas = canvasRef.current
    const container = containerRef.current
    if (!canvas || !container) return
    const ratio = window.devicePixelRatio || 1
    const width = Math.max(1, Math.round(container.clientWidth * ratio))
    const height = Math.max(1, Math.round(container.clientHeight * ratio))
    if (canvas.width === width && canvas.height === height) return
    canvas.width = width
    canvas.height = height
    const s = glRef.current
    if (s?.fitted) s.dist = pcdCameraDistance(s.radius, width / height)
    draw()
  }, [draw])

  useEffect(() => {
    onPickRef.current = onPick
  }, [onPick])

  useEffect(() => {
    pickedRef.current = canPick ? pickedPoint ?? null : null
    updatePickScreen()
  }, [canPick, pickedPoint, updatePickScreen])

  useEffect(() => {
    const container = containerRef.current
    if (!container) return
    const observer = new ResizeObserver(resizeCanvas)
    observer.observe(container)
    resizeCanvas()
    return () => observer.disconnect()
  }, [resizeCanvas])

  useEffect(() => {
    const observer = new MutationObserver(() => {
      const s = glRef.current
      const canvas = canvasRef.current
      if (!s || !canvas) return
      s.colors = heightColors(canvas)
      draw()
    })
    observer.observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] })
    return () => observer.disconnect()
  }, [draw])

  useEffect(() => {
    if (recordingMode) return
    const controller = new AbortController()
    let scene: GLS | null = null
    pickedRef.current = null
    dragRef.current = null
    setPickScreen(null)
    onPickRef.current?.(null)
    setInfo(null)
    setCeiling(null)
    setErrorMessage('')
    setViewMode('2d')
    setStatus(mapName ? 'loading' : 'idle')

    const load = async (name: string) => {
      try {
        const preview = await fetchMapPreview(name, contentEpoch, controller.signal)
        const buffer = preview.buffer
        if (controller.signal.aborted) return
        const pts = parsePcd(buffer)
        if (!pts || pts.length < 3) throw new Error('PCD 为空或格式不支持')
        const canvas = canvasRef.current!
        resizeCanvas()
        const uploaded = uploadCloudScene(canvas, pts, appearanceRef.current)
        scene = uploaded.scene
        glRef.current = scene
        draw()
        setCeiling(uploaded.maxZ)
        setInfo({ count: pts.length/3, visibleCount: pts.length/3, originalCount: pcdPointCount(buffer),
          width: uploaded.width, depth: uploaded.depth, minZ: uploaded.minZ, maxZ: uploaded.maxZ, gridStep: uploaded.gridStep,
          contentEpoch: preview.contentEpoch, frameId: preview.frameId })
        setStatus('done')
      } catch (cause) {
        if (!controller.signal.aborted) {
          setErrorMessage(cause instanceof Error ? cause.message : '地图点云读取失败')
          setStatus('error')
        }
      }
    }
    if (mapName) void load(mapName)
    return () => {
      controller.abort()
      if (scene) disposeCloudScene(scene)
      if (glRef.current === scene) glRef.current = null
    }
  }, [mapName, contentEpoch, productSessionId, refreshToken, draw, resizeCanvas, recordingMode])

  useEffect(() => {
    if (!recording) return
    let current = true
    void Promise.resolve().then(() => {
      if (!current || !canvasRef.current) return
      try {
        resizeCanvas()
        const uploaded = uploadCloudScene(canvasRef.current, recording.points, appearanceRef.current, glRef.current ?? undefined, recording.lines)
        glRef.current = uploaded.scene
        draw()
        setCeiling(uploaded.scene.maxVisibleZ)
        setInfo({ count: recording.points.length / 3, visibleCount: recording.points.length / 3, originalCount: recording.originalCount,
          width: uploaded.width, depth: uploaded.depth, minZ: uploaded.minZ, maxZ: uploaded.maxZ, gridStep: uploaded.gridStep,
          contentEpoch: 0, frameId: recording.frameId })
        setStatus('done')
      } catch (cause: unknown) { setErrorMessage(cause instanceof Error ? cause.message : '三维数据读取失败'); setStatus('error') }
    })
    return () => { current = false }
  }, [recording, draw, resizeCanvas])

  useEffect(() => () => {
    if (recordingMode && glRef.current) { disposeCloudScene(glRef.current); glRef.current = null }
  }, [recordingMode])

  const pickNearest = useCallback((e: React.MouseEvent) => {
    const s = glRef.current
    const canvas = canvasRef.current
    if (!s || !canvas || !onPick || s.style.opacity === 0) return
    const rect = canvas.getBoundingClientRect()
    const px = (e.clientX - rect.left) * (canvas.width / rect.width)
    const py = (e.clientY - rect.top) * (canvas.height / rect.height)
    const [minSize, maxSize] = s.gl.getParameter(s.gl.ALIASED_POINT_SIZE_RANGE) as Float32Array
    const pointSize = Math.max(minSize, Math.min(maxSize, s.style.pointSize * (window.devicePixelRatio || 1)))
    const point = pickVisiblePcdPoint(s.pts, mvpFor(s), canvas.width, canvas.height,
      px, py, 18 * (canvas.width / rect.width), pointSize, s.maxVisibleZ)
    if (!point) return
    pickedRef.current = point
    onPick?.(point)
    updatePickScreen(point)
  }, [onPick, updatePickScreen])

  const onDown  = (e: React.MouseEvent) => {
    dragRef.current = { x: e.clientX, y: e.clientY, startX: e.clientX, startY: e.clientY, moved: false }
  }
  const onUp    = (e: React.MouseEvent) => {
    const drag = dragRef.current
    dragRef.current = null
    if (onPick && drag && !drag.moved && e.button === 0) pickNearest(e)
  }
  const onMove  = (e: React.MouseEvent) => {
    const s = glRef.current; if (!s || !dragRef.current) return
    const dx=e.clientX-dragRef.current.x, dy=e.clientY-dragRef.current.y
    if (Math.hypot(e.clientX - dragRef.current.startX, e.clientY - dragRef.current.startY) > 3) {
      dragRef.current.moved = true
    }
    dragRef.current.x = e.clientX
    dragRef.current.y = e.clientY
    if (s.mode === '2d') {
      const height = (s.gl.canvas as HTMLCanvasElement).clientHeight
      const unitsPerPixel = 2 * s.dist * Math.tan(Math.PI/8) / Math.max(1, height)
      s.center[2] -= dx * unitsPerPixel
      s.center[0] += dy * unitsPerPixel
      s.fitted = false
    } else {
      s.theta -= dx*0.007
      s.phi = Math.max(0.05, Math.min(Math.PI-0.05, s.phi+dy*0.007))
    }
    draw()
  }
  const onWheel = (e: React.WheelEvent) => {
    const s = glRef.current; if (!s) return
    const canvas = s.gl.canvas as HTMLCanvasElement
    const maxDistance = Math.max(s.radius*10, pcdCameraDistance(s.radius, canvas.width/canvas.height)*4)
    s.fitted = false
    s.dist = Math.max(s.radius*0.3, Math.min(maxDistance, s.dist*(1+e.deltaY*0.001)))
    draw()
  }
  const onFit = () => {
    const s = glRef.current; if (!s) return
    const canvas = s.gl.canvas as HTMLCanvasElement
    s.fitted = true
    s.center = [...s.mapCenter]
    s.dist = pcdCameraDistance(s.radius, canvas.width/canvas.height)
    draw()
  }
  const changeView = (mode: ViewMode) => {
    const s = glRef.current; if (!s) return
    setViewMode(mode)
    s.mode = mode
    s.theta = mode === '2d' ? -Math.PI/2 : 0.4
    s.phi = mode === '2d' ? 0 : 1.05
    onFit()
  }

  const changeCeiling = (value: number) => {
    const s = glRef.current
    if (!s) return
    s.maxVisibleZ = value
    setCeiling(value)
    let visibleCount = 0
    for (let i = 2; i < s.pts.length; i += 3) if (s.pts[i] <= value) visibleCount += 1
    setInfo(current => current ? { ...current, visibleCount } : current)
    if (pickedRef.current && pickedRef.current.z > value) {
      pickedRef.current = null
      onPickRef.current?.(null)
    }
    draw()
  }

  const palette = cloudStyleColors(appearance)
  const background = appearance.background === 'theme' ? 'var(--bg)'
    : appearance.background === 'light' ? '#f5f5f5'
    : appearance.background === 'dark' ? '#101820' : appearance.backgroundColor
  const appearanceCss = {
    '--cloud-background': background,
    ...(palette ? { '--cloud-low': palette.lo.join(', '), '--cloud-high': palette.hi.join(', ') } : {}),
  } as CSSProperties
  const changeStyle = (patch: Partial<CloudStyle>) => setAppearance(current => ({ ...current, ...patch }))
  const monochrome = appearance.palette === 'mono'
  const backgroundLabel = { theme: '随主题', light: '浅色', dark: '深色', custom: '自定义' }[appearance.background]
  const changeColoring = (mono: boolean) => {
    if (mono && !monochrome) heightPaletteRef.current = appearance.palette
    changeStyle({ palette: mono ? 'mono' : heightPaletteRef.current })
  }
  const closeSettings = () => {
    setSettingsOpen(false)
    settingsButtonRef.current?.focus()
  }

  return (
    <div className={styles.viewer} style={appearanceCss}
      onKeyDown={event => {
        if (event.key !== 'Escape' || !settingsOpen) return
        closeSettings()
      }}>
      {status === 'done' && <div className={styles.controls} role="group" aria-label="点云视角">
        <div className={styles.viewModes}>
          <button type="button" aria-pressed={viewMode === '2d'} aria-label="2D 俯视" onClick={() => changeView('2d')}>2D</button>
          <button type="button" aria-pressed={viewMode === '3d'} aria-label="3D 旋转视图" onClick={() => changeView('3d')}>3D</button>
        </div>
        <button type="button" className={styles.fitBtn} onClick={onFit} aria-label="显示整图" title="显示整图"><Maximize2 size={16} /></button>
        <button type="button" ref={settingsButtonRef} className={styles.settingsToggle} onClick={() => {
          if (settingsOpen) closeSettings()
          else setSettingsOpen(true)
        }} aria-label="显示设置" title="显示设置" aria-expanded={settingsOpen} aria-controls={settingsId}>
          <SlidersHorizontal size={16} />
        </button>
      </div>}
      {!recordingMode && status === 'done' && info && ceiling != null && <div className={styles.heightControl} role="group" aria-label="显示高度"
          title="只显示地图 Z 不高于此值的点云，不改变地图或导航高度">
          <span>显示高度</span>
          <input type="range" className={styles.slider} aria-label="显示高度" min={info.minZ} max={info.maxZ}
            style={{ '--range-progress': `${info.maxZ > info.minZ ? (ceiling - info.minZ) / (info.maxZ - info.minZ) * 100 : 0}%` } as CSSProperties}
            step="any" value={ceiling}
            onChange={event => changeCeiling(Number(event.target.value))} />
          <label className={styles.heightValue} title="输入米数，按回车或移开焦点应用">
            <input key={ceiling} type="number" inputMode="decimal" step="any" aria-label="显示高度上限（米）"
              defaultValue={ceiling.toFixed(2)}
              onBlur={event => {
                const input = event.currentTarget
                if (input.value === ceiling.toFixed(2)) return
                const value = input.valueAsNumber
                const next = Number.isFinite(value) ? Math.min(info.maxZ, Math.max(info.minZ, value)) : ceiling
                input.value = next.toFixed(2)
                if (next !== ceiling) changeCeiling(next)
              }}
              onKeyDown={event => {
                if (event.key !== 'Enter' && event.key !== 'Escape') return
                event.preventDefault()
                event.stopPropagation()
                if (event.key === 'Escape') event.currentTarget.value = ceiling.toFixed(2)
                event.currentTarget.blur()
              }} />
            <span>m</span>
          </label>
      </div>}
      <div className={styles.body} ref={containerRef}>
        {status === 'idle'    && <div className={styles.placeholder}>{recordingMode ? '正在读取录制场景…' : '选择已保存地图，查看整图点云'}</div>}
        {status === 'loading' && <div className={styles.placeholder} role="status">加载中…</div>}
        {status === 'error'   && <div className={styles.placeholder} role="alert">加载失败<br /><small>{errorMessage}</small></div>}
        <canvas
          ref={canvasRef}
          aria-label={recordingMode ? '录制点云与轨迹' : mapName ? `${mapName} 整图点云` : '整图点云'}
          className={`${styles.canvas} ${onPick ? styles.pickable : ''}`}
          style={{ display: status==='done'?'block':'none' }}
          onMouseDown={onDown} onMouseMove={onMove} onMouseUp={onUp} onMouseLeave={() => { dragRef.current = null }}
          onWheel={onWheel}
        />
        {onPick && pickScreen && <div className={styles.pickMarker} style={{ left: pickScreen.x, top: pickScreen.y }} />}
      </div>
      {status === 'done' && <>
        <section id={settingsId} className={styles.settingsPanel} aria-label="点云显示设置" hidden={!settingsOpen}>
          <header><strong>点云显示</strong><div>
            <button type="button" className={styles.resetButton} onClick={() => {
              heightPaletteRef.current = DEFAULT_CLOUD_STYLE.palette
              setAppearance({ ...DEFAULT_CLOUD_STYLE })
            }} aria-label="恢复默认显示" title="恢复默认显示"><RotateCcw size={14} /></button>
            <button type="button" onClick={closeSettings} aria-label="关闭显示设置"><X size={16} /></button>
          </div></header>
          <fieldset className={styles.displaySection}>
            <legend>颜色</legend>
            <fieldset className={styles.coloringModes} aria-label="着色方式">
              <label className={styles.coloringOption} data-selected={!monochrome}>
                <input type="radio" name={`${settingsId}-coloring`} checked={!monochrome} onChange={() => changeColoring(false)} /><span>高度 Z</span>
              </label>
              <label className={styles.coloringOption} data-selected={monochrome}>
                <input type="radio" name={`${settingsId}-coloring`} checked={monochrome} onChange={() => changeColoring(true)} /><span>单色</span>
              </label>
            </fieldset>
            {monochrome ? <label className={styles.settingRow}><span>颜色</span>
              <input type="color" aria-label="单色颜色" value={appearance.monoColor} onInput={event => changeStyle({ monoColor: event.currentTarget.value })} />
            </label> : <>
              <label className={styles.paletteRow}><span>色带</span><span className={styles.paletteSelect}>
                <select aria-label="配色方案" value={appearance.palette} onChange={event => changeStyle({ palette: event.target.value as CloudStyle['palette'] })}>
                  {CLOUD_PALETTES.filter(option => option.id !== 'mono').map(option => <option key={option.id} value={option.id}>
                    {option.id === 'theme' ? '随主题' : option.label}
                  </option>)}
                </select><ChevronDown size={13} aria-hidden="true" />
              </span></label>
              {info && <div className={styles.colorScale}>
                <div className={styles.scaleRamp} aria-hidden="true" />
                <div className={styles.scaleEnds}>
                  <label className={styles.scaleEnd}>
                    {appearance.palette === 'custom' && <input type="color" aria-label="低处颜色" value={appearance.lowColor} onInput={event => changeStyle({ lowColor: event.currentTarget.value })} />}
                    <span>{info.minZ.toFixed(2)} m</span>
                  </label>
                  <label className={styles.scaleEnd}>
                    <span>{info.maxZ.toFixed(2)} m</span>
                    {appearance.palette === 'custom' && <input type="color" aria-label="高处颜色" value={appearance.highColor} onInput={event => changeStyle({ highColor: event.currentTarget.value })} />}
                  </label>
                </div>
              </div>}
            </>}
          </fieldset>
          <fieldset className={styles.displaySection}>
            <legend>点渲染</legend>
            <label className={styles.rangeRow}><span>点大小</span><input type="range" className={styles.slider} min="0.5" max="6" step="0.1" value={appearance.pointSize}
              style={{ '--range-progress': `${(appearance.pointSize - 0.5) / 5.5 * 100}%` } as CSSProperties}
              onChange={event => changeStyle({ pointSize: Number(event.target.value) })} /><output>{appearance.pointSize.toFixed(1)} px</output></label>
            <label className={styles.rangeRow}><span>不透明度</span><input type="range" className={styles.slider} min="0" max="100" step="1" value={Math.round(appearance.opacity * 100)}
              style={{ '--range-progress': `${Math.round(appearance.opacity * 100)}%` } as CSSProperties}
              onChange={event => changeStyle({ opacity: Number(event.target.value) / 100 })} /><output>{Math.round(appearance.opacity * 100)}%</output></label>
          </fieldset>
          <details className={styles.moreDisplay}>
            <summary><span>场景</span><span className={styles.sceneSummaryValue}>{backgroundLabel}<ChevronDown size={13} /></span></summary>
            <fieldset className={styles.choices}>
              <legend>背景</legend>
              <div className={styles.backgroundChoices}>
                {([{ id: 'theme', label: '随主题' }, { id: 'light', label: '浅色' }, { id: 'dark', label: '深色' }, { id: 'custom', label: '自定义' }] as const).map(option =>
                  <label key={option.id} className={styles.backgroundOption} data-selected={appearance.background === option.id}>
                    <input type="radio" name={`${settingsId}-background`} value={option.id} checked={appearance.background === option.id}
                      onChange={() => changeStyle({ background: option.id })} />
                    <span>{option.label}</span>
                  </label>)}
              </div>
            </fieldset>
            {appearance.background === 'custom' && <label className={styles.settingRow}><span>背景色</span><input type="color" aria-label="背景色" value={appearance.backgroundColor}
              onInput={event => changeStyle({ backgroundColor: event.currentTarget.value })} /></label>}
            <label className={styles.settingRow}><span>坐标网格</span><input type="checkbox" checked={appearance.grid} onChange={event => changeStyle({ grid: event.target.checked })} /></label>
          </details>
        </section>
        {!recordingMode && info && <details className={styles.legend} key={mapName}>
                <summary aria-label="地图信息"><span className={styles.legendSwatch} />{appearance.palette === 'mono' ? '单色' : '高度着色'}<ChevronUp size={13} /></summary>
                <div className={styles.legendDetail}>
                  <div className={styles.heightRange}><span>{info.minZ.toFixed(1)} m</span><span>{info.maxZ.toFixed(1)} m</span></div>
                  <div className={styles.legendRamp} />
                  <p>{appearance.palette === 'mono' ? '统一颜色，不表示语义类别' : '颜色仅表示高度'}</p>
                  <dl>
                    <div><dt>水平范围</dt><dd>{info.width.toFixed(1)} × {info.depth.toFixed(1)} m</dd></div>
                    <div><dt>当前显示</dt><dd>{info.visibleCount.toLocaleString('zh-CN')} / {info.count.toLocaleString('zh-CN')} 点</dd></div>
                    <div><dt>原始点数</dt><dd>{info.originalCount?.toLocaleString('zh-CN') ?? '未知'}</dd></div>
                    <div><dt>地图版本</dt><dd>v{info.contentEpoch} · {info.frameId}</dd></div>
                    <div><dt>网格间距</dt><dd>{Number(info.gridStep.toPrecision(2))} m</dd></div>
                  </dl>
                  <p>网格位于点云最低高度，仅作坐标参考。</p>
                  <p>{onPick ? '点击选点 · ' : ''}{viewMode === '2d' ? '拖拽平移' : '拖拽旋转'} · 滚轮缩放</p>
                </div>
              </details>
          }
      </>}
    </div>
  )
}
