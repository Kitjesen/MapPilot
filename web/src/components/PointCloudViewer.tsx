import { useEffect, useRef, useState, useCallback } from 'react'
import { ChevronUp, Maximize2 } from 'lucide-react'
import styles from './PointCloudViewer.module.css'
import { parsePcd, pcdCameraDistance } from '../services/pcdPreview.ts'
import { PCD_POINT_SIZE_CSS, pickVisiblePcdPoint, projectPcdPoint, type PointCloudPick } from '../services/pcdPicking.ts'

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
  void main() {
    // LingTu X-forward/Y-left/Z-up -> Three X/Y-up/Z (x,z,-y).
    gl_Position = u_mvp * vec4(a_pos.x, a_pos.z, -a_pos.y, 1.0);
    gl_PointSize = u_pointSize;
    v_t = clamp((a_pos.z - u_minZ) / max(u_rangeZ, 0.01), 0.0, 1.0);
  }
`
const FS = `
  precision mediump float;
  varying float v_t;
  uniform vec3 u_lo;
  uniform vec3 u_hi;
  uniform bool u_points;
  void main() {
    if (u_points && distance(gl_PointCoord, vec2(0.5)) > 0.5) discard;
    gl_FragColor = u_points
      ? vec4(mix(u_lo, u_hi, v_t), 1.0)
      : vec4(u_lo, 0.16);
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
  center: [number,number,number]; radius: number
  mapCenter: [number,number,number]
  theta: number; phi: number; dist: number
  fitted: boolean
  mode: ViewMode
  colors: HeightColors
}

interface CloudInfo {
  count: number; width: number; depth: number; minZ: number; maxZ: number; gridStep: number
}

// ── Component ─────────────────────────────────────────────────
export function PointCloudViewer({
  mapName,
  pickedPoint,
  onPick,
}: {
  mapName: string | null
  pickedPoint?: PointCloudPick | null
  onPick?: (point: PointCloudPick | null) => void
}) {
  const canvasRef    = useRef<HTMLCanvasElement>(null)
  const containerRef = useRef<HTMLDivElement>(null)
  const glRef        = useRef<GLS | null>(null)
  const dragRef      = useRef<{ x: number; y: number; startX: number; startY: number; moved: boolean } | null>(null)
  const pickedRef    = useRef<PointCloudPick | null>(null)
  const onPickRef    = useRef(onPick)

  const [status,       setStatus      ] = useState<'idle'|'loading'|'done'|'error'>('idle')
  const [info,         setInfo        ] = useState<CloudInfo | null>(null)
  const [viewMode,     setViewMode    ] = useState<ViewMode>('2d')
  const [pickScreen,   setPickScreen  ] = useState<{ x: number; y: number } | null>(null)
  const canPick = onPick !== undefined

  const updatePickScreen = useCallback((point = pickedRef.current) => {
    const s = glRef.current
    const canvas = canvasRef.current
    if (!s || !canvas || !point || !onPickRef.current) {
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
    gl.uniform1f(gl.getUniformLocation(prog,'u_pointSize'), PCD_POINT_SIZE_CSS * (window.devicePixelRatio || 1))
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
    gl.drawArrays(gl.LINES, 0, gridCount)
    gl.depthMask(true)
    gl.disable(gl.BLEND)
    gl.bindBuffer(gl.ARRAY_BUFFER, vbuf)
    gl.vertexAttribPointer(loc, 3, gl.FLOAT, false, 0, 0)
    gl.uniform1i(gl.getUniformLocation(prog,'u_points'), 1)
    gl.drawArrays(gl.POINTS, 0, nPts)
    updatePickScreen()
  }, [updatePickScreen])

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
    const controller = new AbortController()
    let scene: GLS | null = null
    pickedRef.current = null
    dragRef.current = null
    setPickScreen(null)
    onPickRef.current?.(null)
    setInfo(null)
    setViewMode('2d')
    setStatus(mapName ? 'loading' : 'idle')

    const load = async (name: string) => {
      try {
        const res = await fetch(`/api/v1/maps/${encodeURIComponent(name)}/pcd`, {
          signal: controller.signal,
        })
        if (!res.ok) throw new Error(`HTTP ${res.status}`)
        const buffer = await res.arrayBuffer()
        if (controller.signal.aborted) return
        const pts = parsePcd(buffer)
        if (!pts || pts.length < 3) throw new Error('parse')
        let x0=Infinity,x1=-Infinity,y0=Infinity,y1=-Infinity,z0=Infinity,z1=-Infinity
        for (let i=0; i<pts.length; i+=3) {
          if(pts[i  ]<x0)x0=pts[i  ]; if(pts[i  ]>x1)x1=pts[i  ]
          if(pts[i+1]<y0)y0=pts[i+1]; if(pts[i+1]>y1)y1=pts[i+1]
          if(pts[i+2]<z0)z0=pts[i+2]; if(pts[i+2]>z1)z1=pts[i+2]
        }
        const radius = Math.max(0.1, Math.hypot(x1-x0, y1-y0, z1-z0) / 2)
        const canvas = canvasRef.current!
        resizeCanvas()
        const gl = canvas.getContext('webgl')
        if (!gl) throw new Error('WebGL unavailable')
        gl.clearColor(0, 0, 0, 0); gl.enable(gl.DEPTH_TEST)
        const prog = compileProg(gl)
        const vbuf = gl.createBuffer()!
        const gridbuf = gl.createBuffer()!
        const grid = referenceGrid(x0, x1, y0, y1, z0)
        const mapCenter: [number, number, number] = [(x0+x1)/2, (z0+z1)/2, -(y0+y1)/2]
        scene = {
          gl, prog, vbuf, nPts: pts.length/3, pts,
          gridbuf, gridCount: grid.points.length/3,
          minZ: z0, rangeZ: z1-z0,
          center: [...mapCenter], mapCenter,
          radius, theta: -Math.PI/2, phi: 0,
          dist: pcdCameraDistance(radius, canvas.width / canvas.height),
          fitted: true, mode: '2d', colors: heightColors(canvas),
        }
        gl.bindBuffer(gl.ARRAY_BUFFER, gridbuf)
        gl.bufferData(gl.ARRAY_BUFFER, grid.points, gl.STATIC_DRAW)
        gl.bindBuffer(gl.ARRAY_BUFFER, vbuf)
        gl.bufferData(gl.ARRAY_BUFFER, pts, gl.STATIC_DRAW)
        glRef.current = scene
        draw()
        setInfo({ count: pts.length/3, width: x1-x0, depth: y1-y0, minZ: z0, maxZ: z1, gridStep: grid.step })
        setStatus('done')
      } catch {
        if (!controller.signal.aborted) setStatus('error')
      }
    }
    if (mapName) void load(mapName)
    return () => {
      controller.abort()
      if (scene) {
        scene.gl.deleteBuffer(scene.vbuf)
        scene.gl.deleteBuffer(scene.gridbuf)
        scene.gl.deleteProgram(scene.prog)
      }
      if (glRef.current === scene) glRef.current = null
    }
  }, [mapName, draw, resizeCanvas])

  const pickNearest = useCallback((e: React.MouseEvent) => {
    const s = glRef.current
    const canvas = canvasRef.current
    if (!s || !canvas || !onPick) return
    const rect = canvas.getBoundingClientRect()
    const px = (e.clientX - rect.left) * (canvas.width / rect.width)
    const py = (e.clientY - rect.top) * (canvas.height / rect.height)
    const [minSize, maxSize] = s.gl.getParameter(s.gl.ALIASED_POINT_SIZE_RANGE) as Float32Array
    const pointSize = Math.max(minSize, Math.min(maxSize, PCD_POINT_SIZE_CSS * (window.devicePixelRatio || 1)))
    const point = pickVisiblePcdPoint(s.pts, mvpFor(s), canvas.width, canvas.height,
      px, py, 18 * (canvas.width / rect.width), pointSize)
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

  return (
    <div className={styles.viewer}>
      <div className={styles.body} ref={containerRef}>
        {status === 'idle'    && <div className={styles.placeholder}>选择已保存地图，查看整图点云</div>}
        {status === 'loading' && <div className={styles.placeholder}>加载中…</div>}
        {status === 'error'   && <div className={styles.placeholder}>加载失败<br /><small>无 PCD 或格式不支持</small></div>}
        <canvas
          ref={canvasRef}
          aria-label={mapName ? `${mapName} 整图点云` : '整图点云'}
          className={`${styles.canvas} ${onPick ? styles.pickable : ''}`}
          style={{ display: status==='done'?'block':'none' }}
          onMouseDown={onDown} onMouseMove={onMove} onMouseUp={onUp} onMouseLeave={() => { dragRef.current = null }}
          onWheel={onWheel}
        />
        {status === 'done' && (
          <>
            <div className={styles.controls} role="group" aria-label="点云视角">
              <div className={styles.viewModes}>
                <button type="button" aria-pressed={viewMode === '2d'} aria-label="2D 俯视" onClick={() => changeView('2d')}>2D</button>
                <button type="button" aria-pressed={viewMode === '3d'} aria-label="3D 旋转视图" onClick={() => changeView('3d')}>3D</button>
              </div>
              <button type="button" className={styles.fitBtn} onClick={onFit} aria-label="显示整图" title="显示整图">
                <Maximize2 size={16} />
              </button>
            </div>
            {info && (
              <details className={styles.legend} key={mapName}>
                <summary><span className={styles.legendSwatch} />高度着色<ChevronUp size={13} /></summary>
                <div className={styles.legendDetail}>
                  <div className={styles.heightRange}><span>{info.minZ.toFixed(1)} m</span><span>{info.maxZ.toFixed(1)} m</span></div>
                  <div className={styles.legendRamp} />
                  <p>颜色仅表示高度</p>
                  <dl>
                    <div><dt>水平范围</dt><dd>{info.width.toFixed(1)} × {info.depth.toFixed(1)} m</dd></div>
                    <div><dt>显示点数</dt><dd>{info.count.toLocaleString('zh-CN')}（采样）</dd></div>
                    <div><dt>网格间距</dt><dd>{Number(info.gridStep.toPrecision(2))} m</dd></div>
                  </dl>
                  <p>网格位于点云最低高度，仅作坐标参考。</p>
                  <p>{onPick ? '点击选点 · ' : ''}{viewMode === '2d' ? '拖拽平移' : '拖拽旋转'} · 滚轮缩放</p>
                </div>
              </details>
            )}
          </>
        )}
        {onPick && pickScreen && <div className={styles.pickMarker} style={{ left: pickScreen.x, top: pickScreen.y }} />}
      </div>
    </div>
  )
}
