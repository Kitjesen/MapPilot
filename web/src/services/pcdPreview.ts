// Include both ends of the map instead of truncating its later regions.
function sampleIndex(index: number, total: number, count: number): number {
  return count <= 1 ? 0 : Math.floor(index * (total - 1) / (count - 1))
}

export function pcdCameraDistance(radius: number, aspect: number): number {
  // Fit the bounding sphere to the narrower field of view, with room at the edges.
  const halfFov = Math.atan(Math.tan(Math.PI / 8) * Math.min(1, aspect))
  return Math.max(0.1, radius) / Math.sin(halfFov) * 1.1
}

export function parsePcd(buffer: ArrayBuffer): Float32Array | null {
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
      const p = lines[sampleIndex(i, lines.length, cap)].trim().split(/\s+/)
      const x = +p[xi], y = +p[yi], z = zi >= 0 ? +p[zi] : 0
      if (isFinite(x) && isFinite(y)) { out[n++]=x; out[n++]=y; out[n++]=z }
    }
    return out.slice(0, n)
  }
  if (fmt.trim() === 'binary') {
    const stride = szs.reduce((a, b) => a + b, 0)
    const offsets: number[] = [0]
    for (let i = 0; i < szs.length - 1; i++) offsets.push(offsets[i] + szs[i])
    const dv = new DataView(buffer, he)
    const available = Math.floor(dv.byteLength / stride)
    const total = Math.min(nPts || available, available)
    const cap = Math.min(total, 500_000)
    const out = new Float32Array(cap * 3)
    let n = 0
    for (let i = 0; i < cap; i++) {
      const b = sampleIndex(i, total, cap) * stride
      const x = dv.getFloat32(b + offsets[xi], true)
      const y = dv.getFloat32(b + offsets[yi], true)
      const z = zi >= 0 ? dv.getFloat32(b + offsets[zi], true) : 0
      if (isFinite(x) && isFinite(y)) { out[n++]=x; out[n++]=y; out[n++]=z }
    }
    return out.slice(0, n)
  }
  return null
}
