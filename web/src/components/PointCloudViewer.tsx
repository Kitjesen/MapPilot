import { useEffect, useRef, useState, useCallback } from 'react'
import { ScanLine, RotateCcw } from 'lucide-react'
import styles from './PointCloudViewer.module.css'

// ── PCD Parser ────────────────────────────────────────────────
function parsePcd(buffer: ArrayBuffer): Float32Array | null {
  const bytes = new Uint8Array(buffer)
  let he = -1, fmt = ''
  for (let i = 0; i < Math.min(bytes.length - 8, 8192); i++) {
    if (bytes[i]===68&&bytes[i+1]===65&&bytes[i+2]===84&&bytes[i+3]===65&&bytes[i+4]===32) {
      let j = i + 5
      while (j < bytes.length && bytes[j] !== 10 && bytes[j] !== 13)
        fmt += String.fromCharCode(bytes[j++])
      if (bytes[j]===13) j++; if (bytes[j]===10) j++
      he = j; break
    }
  }
  if (he < 0) return null

  const hdr  = new TextDecoder().decode(bytes.slice(0, he))
  const flds = hdr.match(/^FIELDS\s+(.+)$/m)?.[1].split(/\s+/) ?? []
  const szs  = hdr.match(/^SIZE\s+(.+)$/m)?.[1].split(/\s+/).map(Number) ?? flds.map(() => 4)
  const nPts = parseInt(hdr.match(/^POINTS\s+(\d+)$/m)?.[1] ?? '0')
  const xi = flds.indexOf('x'), yi = flds.indexOf('y'), zi = flds.indexOf('z')
  if (xi < 0 || yi < 0) return null

  if (fmt.trim() === 'ascii') {
    const lines = new TextDecoder().decode(bytes.slice(he)).trim().split('\n')
    const cap = Math.min(lines.length, 300_000), out = new Float32Array(cap * 3)
    let n = 0
    for (let i = 0; i < cap; i++) {
      const p = lines[i].trim().split(/\s+/)
      const x = +p[xi], y = +p[yi], z = zi >= 0 ? +p[zi] : 0
      if (isFinite(x) && isFinite(y)) { out[n++]=x; out[n++]=y; out[n++]=z }
    }
    return out.slice(0, n)
  }
  if (fmt.trim() === 'binary') {
    const stride = szs.reduce((a, b) => a + b, 0)
    const offsets: number[] = [0]
    for (let i = 0; i < szs.length - 1; i++) offsets.push(offsets[i] + szs[i])
    const dv  = new DataView(buffer, he)
    const tot = Math.min(nPts || Math.floor(dv.byteLength / stride), 500_000)
    const out = new Float32Array(tot * 3)
    let n = 0
    for (let i = 0; i < tot; i++) {
      const b = i * stride; if (b + stride > dv.byteLength) break
      const x = dv.getFloat32(b + offsets[xi], true)
      const y = dv.getFloat32(b + offsets[yi], true)
      const z = zi >= 0 ? dv.getFloat32(b + offsets[zi], true) : 0
      if (isFinite(x) && isFinite(y)) { out[n++]=x; out[n++]=y; out[n++]=z }
    }
    return out.slice(0, n)
  }
  return null
}

// ── Color schemes ─────────────────────────────────────────────
interface CS { lo: [number,number,number]; hi: [number,number,number] }
const SCHEMES: Record<string, CS> = {
  '高度': { lo: [0.00, 0.38, 0.85], hi: [0.00, 1.00, 0.53] }, // blue → green
  '热力': { lo: [0.10, 0.10, 0.90], hi: [1.00, 0.22, 0.00] }, // blue → red
  '彩虹': { lo: [0.55, 0.00, 0.90], hi: [1.00, 0.80, 0.00] }, // purple → yellow
  '灰度': { lo: [0.18, 0.18, 0.18], hi: [0.90, 0.90, 0.90] }, // dark → light
}

// ── WebGL shaders ─────────────────────────────────────────────
const VS = `
  attribute vec3 a_pos;
  uniform mat4 u_mvp;
  uniform float u_minZ;
  uniform float u_rangeZ;
  varying float v_t;
  void main() {
    gl_Position = u_mvp * vec4(a_pos.x, a_pos.z, a_pos.y, 1.0);
    gl_PointSize = 1.8;
    v_t = clamp((a_pos.z - u_minZ) / max(u_rangeZ, 0.01), 0.0, 1.0);
  }
`
const FS = `
  precision mediump float;
  varying float v_t;
  uniform vec3 u_lo;
  uniform vec3 u_hi;
  void main() {
    gl_FragColor = vec4(mix(u_lo, u_hi, v_t), 1.0);
  }
`

function compileProg(gl: WebGLRenderingContext) {
  const compile = (type: number, src: string) => {
    const s = gl.createShader(type)!
    gl.shaderSource(s, src); gl.compileShader(s)
    if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s) ?? '')
    return s
  }
  const p = gl.createProgram()!
  gl.attachShader(p, compile(gl.VERTEX_SHADER,   VS))
  gl.attachShader(p, compile(gl.FRAGMENT_SHADER, FS))
  gl.linkProgram(p)
  if (!gl.getProgramParameter(p, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(p) ?? '')
  return p
}

// ── Mat4 helpers (column-major) ────────────────────────────────
function perspective(fov: number, aspect: number, near: number, far: number) {
  const f = 1/Math.tan(fov/2), nf = 1/(near-far), m = new Float32Array(16)
  m[0]=f/aspect; m[5]=f; m[10]=(far+near)*nf; m[11]=-1; m[14]=2*far*near*nf
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

export interface PointCloudPick { x: number; y: number; z: number }

function mvpFor(s: GLS) {
  const c = s.gl.canvas as HTMLCanvasElement
  const sinP=Math.sin(s.phi), cosP=Math.cos(s.phi)
  const ex=s.center[0]+s.dist*sinP*Math.cos(s.theta)
  const ey=s.center[1]+s.dist*cosP
  const ez=s.center[2]+s.dist*sinP*Math.sin(s.theta)
  return mulM(
    perspective(Math.PI/4, c.width/c.height, 0.05, s.dist*20),
    lookAt(ex,ey,ez, s.center[0],s.center[1],s.center[2]),
  )
}

function projectPcdPoint(m: Float32Array, p: PointCloudPick, width: number, height: number) {
  const x = p.x, y = p.z, z = p.y
  const cx = m[0]*x + m[4]*y + m[8] *z + m[12]
  const cy = m[1]*x + m[5]*y + m[9] *z + m[13]
  const cz = m[2]*x + m[6]*y + m[10]*z + m[14]
  const cw = m[3]*x + m[7]*y + m[11]*z + m[15]
  if (!isFinite(cw) || cw <= 0) return null
  const nx = cx / cw, ny = cy / cw, nz = cz / cw
  if (nx < -1 || nx > 1 || ny < -1 || ny > 1 || nz < -1 || nz > 1) return null
  return { x: (nx * 0.5 + 0.5) * width, y: (0.5 - ny * 0.5) * height }
}

// ── GL state ──────────────────────────────────────────────────
interface GLS {
  gl: WebGLRenderingContext; prog: WebGLProgram; vbuf: WebGLBuffer; nPts: number
  pts: Float32Array
  minZ: number; rangeZ: number
  center: [number,number,number]; radius: number
  theta: number; phi: number; dist: number
  scheme: CS
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

  const [status,       setStatus      ] = useState<'idle'|'loading'|'done'|'error'>('idle')
  const [info,         setInfo        ] = useState('')
  const [schemeName,   setSchemeName  ] = useState('高度')
  const [pickScreen,   setPickScreen  ] = useState<{ x: number; y: number } | null>(null)

  const updatePickScreen = useCallback((point = pickedRef.current) => {
    const s = glRef.current
    const canvas = canvasRef.current
    if (!s || !canvas || !point) {
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
    const { gl, prog, vbuf, nPts, minZ, rangeZ, scheme } = s
    const c = gl.canvas as HTMLCanvasElement
    gl.viewport(0, 0, c.width, c.height)
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)
    const mvp = mvpFor(s)
    gl.useProgram(prog)
    gl.uniformMatrix4fv(gl.getUniformLocation(prog,'u_mvp'), false, mvp)
    gl.uniform1f(gl.getUniformLocation(prog,'u_minZ'), minZ)
    gl.uniform1f(gl.getUniformLocation(prog,'u_rangeZ'), rangeZ)
    gl.uniform3fv(gl.getUniformLocation(prog,'u_lo'), scheme.lo)
    gl.uniform3fv(gl.getUniformLocation(prog,'u_hi'), scheme.hi)
    gl.bindBuffer(gl.ARRAY_BUFFER, vbuf)
    const loc = gl.getAttribLocation(prog,'a_pos')
    gl.enableVertexAttribArray(loc)
    gl.vertexAttribPointer(loc, 3, gl.FLOAT, false, 0, 0)
    gl.drawArrays(gl.POINTS, 0, nPts)
    updatePickScreen()
  }, [updatePickScreen])

  const load = useCallback(async (name: string, scheme: CS) => {
    glRef.current = null
    pickedRef.current = null
    setPickScreen(null)
    onPick?.(null)
    setStatus('loading')
    setInfo('')
    try {
      const res = await fetch(`/api/v1/maps/${encodeURIComponent(name)}/pcd`)
      if (!res.ok) throw new Error(`HTTP ${res.status}`)
      const pts = parsePcd(await res.arrayBuffer())
      if (!pts || pts.length < 3) throw new Error('parse')
      let x0=Infinity,x1=-Infinity,y0=Infinity,y1=-Infinity,z0=Infinity,z1=-Infinity
      for (let i=0; i<pts.length; i+=3) {
        if(pts[i  ]<x0)x0=pts[i  ]; if(pts[i  ]>x1)x1=pts[i  ]
        if(pts[i+1]<y0)y0=pts[i+1]; if(pts[i+1]>y1)y1=pts[i+1]
        if(pts[i+2]<z0)z0=pts[i+2]; if(pts[i+2]>z1)z1=pts[i+2]
      }
      const radius = Math.max(x1-x0, y1-y0, z1-z0) / 2
      const canvas = canvasRef.current!
      canvas.width  = containerRef.current?.offsetWidth  || 640
      canvas.height = containerRef.current?.offsetHeight || 480
      const gl = canvas.getContext('webgl')!
      gl.clearColor(0.03, 0.05, 0.09, 1); gl.enable(gl.DEPTH_TEST)
      const prog  = compileProg(gl)
      const vbuf  = gl.createBuffer()!
      gl.bindBuffer(gl.ARRAY_BUFFER, vbuf)
      gl.bufferData(gl.ARRAY_BUFFER, pts, gl.STATIC_DRAW)
      glRef.current = {
        gl, prog, vbuf, nPts: pts.length/3,
        pts,
        minZ: z0, rangeZ: z1-z0,
        center: [(x0+x1)/2, (z0+z1)/2, (y0+y1)/2],
        radius, theta: 0.4, phi: 1.05, dist: radius*2.8, scheme,
      }
      draw()
      setInfo(`${((pts.length/3)/1000).toFixed(0)}k pts`)
      setStatus('done')
    } catch { setStatus('error') }
  }, [draw, onPick])

  useEffect(() => {
    if (mapName) load(mapName, SCHEMES[schemeName] ?? SCHEMES['高度'])
    else { glRef.current = null; setStatus('idle'); setInfo('') }
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [mapName, load])

  useEffect(() => {
    pickedRef.current = pickedPoint ?? null
    updatePickScreen()
  }, [pickedPoint, updatePickScreen])

  const changeScheme = (name: string) => {
    setSchemeName(name)
    const s = glRef.current
    if (s) { s.scheme = SCHEMES[name]; draw() }
  }

  const pickNearest = useCallback((e: React.MouseEvent) => {
    const s = glRef.current
    const canvas = canvasRef.current
    if (!s || !canvas) return
    const rect = canvas.getBoundingClientRect()
    const px = (e.clientX - rect.left) * (canvas.width / rect.width)
    const py = (e.clientY - rect.top) * (canvas.height / rect.height)
    const mvp = mvpFor(s)
    const maxDist = 18 * (canvas.width / rect.width)
    let bestIndex = -1
    let bestD2 = maxDist * maxDist
    for (let i = 0; i < s.pts.length; i += 3) {
      const point = { x: s.pts[i], y: s.pts[i + 1], z: s.pts[i + 2] }
      const projected = projectPcdPoint(mvp, point, canvas.width, canvas.height)
      if (!projected) continue
      const dx = projected.x - px
      const dy = projected.y - py
      const d2 = dx * dx + dy * dy
      if (d2 < bestD2) {
        bestD2 = d2
        bestIndex = i
      }
    }
    if (bestIndex < 0) return
    const point = { x: s.pts[bestIndex], y: s.pts[bestIndex + 1], z: s.pts[bestIndex + 2] }
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
    if (drag && !drag.moved && e.button === 0) pickNearest(e)
  }
  const onMove  = (e: React.MouseEvent) => {
    const s = glRef.current; if (!s || !dragRef.current) return
    const dx=e.clientX-dragRef.current.x, dy=e.clientY-dragRef.current.y
    if (Math.hypot(e.clientX - dragRef.current.startX, e.clientY - dragRef.current.startY) > 3) {
      dragRef.current.moved = true
    }
    dragRef.current.x = e.clientX
    dragRef.current.y = e.clientY
    s.theta -= dx*0.007
    s.phi = Math.max(0.05, Math.min(Math.PI-0.05, s.phi+dy*0.007))
    draw()
  }
  const onWheel = (e: React.WheelEvent) => {
    const s = glRef.current; if (!s) return
    s.dist = Math.max(s.radius*0.3, Math.min(s.radius*10, s.dist*(1+e.deltaY*0.001)))
    draw()
  }
  const onReset = () => {
    const s = glRef.current; if (!s) return
    s.theta=0.4; s.phi=1.05; s.dist=s.radius*2.8; draw()
  }

  return (
    <div className={styles.viewer}>
      <div className={styles.header}>
        <span className={styles.title}><ScanLine size={13} /> 3D 点云</span>
        {mapName && <span className={styles.mapLabel}>{mapName}</span>}
        {info    && <span className={styles.info}>{info}</span>}
        {status === 'done' && (
          <>
            <div className={styles.schemeGroup}>
              {Object.keys(SCHEMES).map(k => (
                <button
                  key={k}
                  className={schemeName === k ? styles.schemeBtnActive : styles.schemeBtn}
                  onClick={() => changeScheme(k)}
                >{k}</button>
              ))}
            </div>
            <button className={styles.resetBtn} onClick={onReset} title="重置视角">
              <RotateCcw size={11} />
            </button>
          </>
        )}
      </div>

      <div className={styles.body} ref={containerRef}>
        {status === 'idle'    && <div className={styles.placeholder}>← 点击「预览」加载 3D 点云</div>}
        {status === 'loading' && <div className={styles.placeholder}>加载中…</div>}
        {status === 'error'   && <div className={styles.placeholder}>加载失败<br /><small>无 PCD 或格式不支持</small></div>}
        <canvas
          ref={canvasRef}
          className={styles.canvas}
          style={{ display: status==='done'?'block':'none', cursor: dragRef.current?'grabbing':'crosshair' }}
          onMouseDown={onDown} onMouseMove={onMove} onMouseUp={onUp} onMouseLeave={() => { dragRef.current = null }}
          onWheel={onWheel}
        />
        {pickScreen && <div className={styles.pickMarker} style={{ left: pickScreen.x, top: pickScreen.y }} />}
      </div>

      {status === 'done' && <div className={styles.hint}>点击选点 · 拖拽旋转 · 滚轮缩放</div>}
    </div>
  )
}
