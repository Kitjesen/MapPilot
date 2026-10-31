import * as THREE from 'three'
import { cloudPointIsVisible } from './layers/layerUtils.ts'
import { pickOctomapSurface } from './layers/octomapLayer.ts'

export type GoalPickMode = 'plane' | 'surface'

/** Picking only proposes a target; the native planner still evaluates the route. */
export function pickNavigationPoint(
  raycaster: THREE.Raycaster,
  clouds: THREE.Points[],
  referenceHeight: number,
  mode: GoalPickMode = 'plane',
  voxels?: THREE.InstancedMesh | null,
): THREE.Vector3 | null {
  if (mode === 'surface') {
    raycaster.params.Points = { threshold: 0.1 }
    const voxelPoint = voxels ? pickOctomapSurface(raycaster, voxels) : null
    for (const hit of raycaster.intersectObjects(clouds, false)) {
      if (voxelPoint && raycaster.ray.origin.distanceTo(voxelPoint) < hit.distance) return voxelPoint
      if (hit.index === undefined) continue
      const points = hit.object as THREE.Points
      // Points raycasting reports the closest point on the ray. Use the actual
      // observed vertex so explicit 3D selection retains its measured height.
      const vertex = new THREE.Vector3()
        .fromBufferAttribute(points.geometry.getAttribute('position'), hit.index)
        .applyMatrix4(points.matrixWorld)
      if (cloudPointIsVisible(points, vertex)) return vertex
    }
    return voxelPoint
  }
  return raycaster.ray.intersectPlane(
    new THREE.Plane(new THREE.Vector3(0, 1, 0), -referenceHeight),
    new THREE.Vector3(),
  )
}
