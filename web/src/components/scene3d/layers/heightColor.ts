import * as THREE from 'three'
import { HEIGHT_COLORS } from '../../../services/cloudHeightColor.ts'

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
      shader.fragmentShader = `varying float colorWorldHeight;\nuniform vec3 heightColorRange;\n${shader.fragmentShader}`.replace(
        '#include <color_fragment>',
        `#include <color_fragment>
if (heightColorRange.z > 0.5) {
  float t = heightColorRange.y > heightColorRange.x
    ? clamp((colorWorldHeight - heightColorRange.x) / (heightColorRange.y - heightColorRange.x), 0.0, 1.0) * 4.0 : 2.0;
  ${HEIGHT_COLORS.map((rgb, i) => `vec3 h${i} = vec3(${rgb.map(v => v.toFixed(2)).join(', ')});`).join('\n  ')}
  diffuseColor.rgb = t < 1.0 ? mix(h0, h1, t) : t < 2.0 ? mix(h1, h2, t - 1.0)
    : t < 3.0 ? mix(h2, h3, t - 2.0) : mix(h3, h4, t - 3.0);
}`, 
      )
    }
    material.customProgramCacheKey = () => `${programKey}:height-color`
    material.needsUpdate = true
  }
  range.set(min, max, Number(enabled))
}
