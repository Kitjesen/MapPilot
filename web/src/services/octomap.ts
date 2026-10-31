export interface Octomap {
  /** Occupied leaf centers and actual edge lengths: x, y, z, size. */
  cells: Float32Array
  resolution: number
  bounds: { min: [number, number, number]; max: [number, number, number] }
}

/** Decode the native full-probability OcTree format; free leaves are not drawn. */
export function parseOctomap(buffer: ArrayBuffer): Octomap {
  const bytes = new Uint8Array(buffer)
  const decoder = new TextDecoder()
  let offset = 0, size = -1, resolution = 0, id = '', data = false
  while (offset < bytes.length) {
    const end = bytes.indexOf(10, offset)
    if (end < 0) break
    const line = decoder.decode(bytes.subarray(offset, end)).trim()
    offset = end + 1
    if (line === 'data') { data = true; break }
    if (line.startsWith('id ')) id = line.slice(3)
    if (line.startsWith('size ')) size = Number(line.slice(5))
    if (line.startsWith('res ')) resolution = Number(line.slice(4))
    if (line.startsWith('# Octomap') && line !== '# Octomap OcTree file') {
      throw new Error('导航体素需要完整概率 OctoMap .ot 文件')
    }
  }
  if (!data || id !== 'OcTree' || !Number.isSafeInteger(size) || size < 0
    || !Number.isFinite(resolution) || resolution <= 0 || bytes.length - offset !== size * 5) {
    throw new Error('OctoMap 文件头或节点长度无效')
  }
  const start = offset, view = new DataView(buffer)
  let nodes = 0, occupied = 0, cells: Float32Array | null = null
  const bounds: Octomap['bounds'] = { min: [Infinity, Infinity, Infinity], max: [-Infinity, -Infinity, -Infinity] }
  function visit(x: number, y: number, z: number, length: number, depth: number): void {
    if (offset + 5 > bytes.length || depth > 16) throw new Error('OctoMap 节点结构无效')
    const logOdds = view.getFloat32(offset, true), children = bytes[offset + 4]
    offset += 5
    nodes++
    if (!Number.isFinite(logOdds)) throw new Error('OctoMap 概率无效')
    if (children) {
      for (let child = 0; child < 8; child++) {
        if (!(children & (1 << child))) continue
        const d = length / 4
        visit(x + (child & 1 ? d : -d), y + (child & 2 ? d : -d),
          z + (child & 4 ? d : -d), length / 2, depth + 1)
      }
    } else if (logOdds >= 0) {
      if (cells) {
        const i = occupied * 4
        cells[i] = x; cells[i + 1] = y; cells[i + 2] = z; cells[i + 3] = length
        for (let axis = 0; axis < 3; axis++) {
          const center = axis === 0 ? x : axis === 1 ? y : z
          bounds.min[axis] = Math.min(bounds.min[axis], center - length / 2)
          bounds.max[axis] = Math.max(bounds.max[axis], center + length / 2)
        }
      }
      occupied++
    }
  }
  if (size) visit(0, 0, 0, resolution * 65536, 0)
  if (!occupied) { bounds.min = [0, 0, 0]; bounds.max = [0, 0, 0] }
  if (nodes !== size || offset !== bytes.length) throw new Error('OctoMap 节点数量不匹配')
  // Count first: the file can contain millions of free/internal nodes that need no GPU storage.
  cells = new Float32Array(occupied * 4)
  offset = start
  occupied = 0
  if (size) visit(0, 0, 0, resolution * 65536, 0)
  return { cells, resolution, bounds }
}
