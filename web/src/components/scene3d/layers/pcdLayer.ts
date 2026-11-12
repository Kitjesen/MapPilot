import * as THREE from 'three'
import type { PcdCloud } from '../../../services/pcdPreview.ts'
import { lingtuToThree } from '../../../services/coordinateFrame.ts'
import { CLOUD_HEIGHT_GLSL } from '../../../services/cloudHeightColor.ts'

export function createPcdLayer(cloud: PcdCloud): THREE.Points<THREE.BufferGeometry, THREE.ShaderMaterial> {
  const geometry = new THREE.BufferGeometry()
  geometry.setAttribute('position', new THREE.BufferAttribute(cloud.positions, 3))
  geometry.boundingBox = new THREE.Box3(new THREE.Vector3(...cloud.bounds.min), new THREE.Vector3(...cloud.bounds.max))
  geometry.boundingSphere = geometry.boundingBox.getBoundingSphere(new THREE.Sphere())
  const material = new THREE.ShaderMaterial({
    clipping: true,
    uniforms: {
      pointSize: { value: 2 }, opacity: { value: .9 },
      heightRange: { value: new THREE.Vector2(cloud.colorRange.min, cloud.colorRange.max) },
    },
    vertexShader: `
      #include <clipping_planes_pars_vertex>
      uniform float pointSize;
      uniform vec2 heightRange;
      varying vec3 mapColor;
      ${CLOUD_HEIGHT_GLSL}
      void main() {
        float ratio = heightRange.y > heightRange.x
          ? (position.z - heightRange.x) / (heightRange.y - heightRange.x) : 0.5;
        mapColor = cloudHeightColor(ratio);
        vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
        gl_Position = projectionMatrix * mvPosition;
        gl_PointSize = pointSize;
        #include <clipping_planes_vertex>
      }`,
    fragmentShader: `
      #include <clipping_planes_pars_fragment>
      uniform float opacity;
      varying vec3 mapColor;
      void main() {
        #include <clipping_planes_fragment>
        gl_FragColor = vec4(mapColor, opacity);
      }`,
    toneMapped: false,
  })
  const layer = new THREE.Points(geometry, material)
  layer.matrix.makeBasis(new THREE.Vector3(...lingtuToThree([1, 0, 0])),
    new THREE.Vector3(...lingtuToThree([0, 1, 0])), new THREE.Vector3(...lingtuToThree([0, 0, 1])))
  layer.matrixAutoUpdate = false
  return layer
}
