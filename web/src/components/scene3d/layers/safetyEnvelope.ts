import * as THREE from 'three'

export interface SafetyEnvelope {
  radius: number
  offset: number
  above: number
  below: number
}

export function safetyEnvelopeFromRunPlan(plan: unknown): SafetyEnvelope | null {
  const config = (plan as { host?: { config?: Record<string, unknown> } } | null)?.host?.config
  if (!config) return null
  const values = [config.collision_cylinder_radius_m, config.collision_cylinder_offset_m,
    config.collision_clearance_above_m, config.collision_clearance_below_m]
  if (!values.every(v => typeof v === 'number' && Number.isFinite(v) && v >= 0)) return null
  const [radius, offset, above, below] = values as number[]
  return radius > 0 && above + below > 0 ? { radius, offset, above, below } : null
}

/** Body-local geometry; the robot group supplies the same interpolated pose. */
export function createSafetyEnvelope(shape: SafetyEnvelope): THREE.Group {
  const group = new THREE.Group()
  group.name = 'robot-safety-envelope'
  for (const sign of [1, -1]) {
    const geometry = new THREE.CylinderGeometry(shape.radius, shape.radius, shape.above + shape.below, 48)
    const mesh = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({
      color: 0x59bfff, transparent: true, opacity: 0.08, depthWrite: false,
    }))
    mesh.position.set(sign * shape.offset, (shape.above - shape.below) / 2, 0)
    group.add(mesh)
    for (const height of [-shape.below, 0, shape.above]) {
      const points = Array.from({ length: 65 }, (_, i) => {
        const angle = i * Math.PI / 32
        return new THREE.Vector3(sign * shape.offset + shape.radius * Math.cos(angle), height,
          shape.radius * Math.sin(angle))
      })
      const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(points),
        new THREE.LineBasicMaterial({ color: 0x59bfff, transparent: true, opacity: height === 0 ? 0.95 : 0.45, depthTest: false }))
      line.renderOrder = 45
      group.add(line)
    }
    const center = new THREE.Mesh(new THREE.SphereGeometry(0.025, 12, 8),
      new THREE.MeshBasicMaterial({ color: 0xffffff, depthTest: false }))
    center.name = sign === 1 ? 'front-query-center' : 'rear-query-center'
    center.position.x = sign * shape.offset
    center.renderOrder = 46
    group.add(center)
  }
  return group
}
