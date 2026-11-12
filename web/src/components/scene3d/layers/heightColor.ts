import * as THREE from 'three'
import { CLOUD_HEIGHT_GLSL } from '../../../services/cloudHeightColor.ts'

const ranges = new WeakMap<THREE.Material, THREE.Vector3>()

/** Change the color scale through uniforms; keep point buffers and clipping intact. */
export function setHeightColorRange(material: THREE.Material, min: number, max: number, enabled: boolean): void {
  let range = ranges.get(material)
  if (!range) {
    range = new THREE.Vector3()
    ranges.set(material, range)
    const uniform = range
    const compile = material.onBeforeCompile
    const programKey = material.customProgramCacheKey()
    material.onBeforeCompile = (shader, renderer) => {
      compile.call(material, shader, renderer)
      shader.uniforms.heightColorRange = { value: uniform }
      shader.vertexShader = `varying float colorWorldHeight;\n${shader.vertexShader}`.replace(
        '#include <project_vertex>',
        `#include <project_vertex>
vec4 colorPosition = vec4(transformed, 1.0);
#ifdef USE_INSTANCING
colorPosition = instanceMatrix * colorPosition;
#endif
colorWorldHeight = (modelMatrix * colorPosition).y;`,
      )
      shader.fragmentShader = `varying float colorWorldHeight;\nuniform vec3 heightColorRange;\n${CLOUD_HEIGHT_GLSL}\n${shader.fragmentShader}`.replace(
        '#include <color_fragment>',
        `#include <color_fragment>
if (heightColorRange.z > 0.5) {
  float t = heightColorRange.y > heightColorRange.x
    ? (colorWorldHeight - heightColorRange.x) / (heightColorRange.y - heightColorRange.x) : 0.5;
  diffuseColor.rgb = cloudHeightColor(t);
}`, 
      )
    }
    material.customProgramCacheKey = () => `${programKey}:height-color`
    material.needsUpdate = true
  }
  range.set(min, max, Number(enabled))
}
