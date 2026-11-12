import { useSurfaceMap } from '../hooks/useSurfaceMap.ts'
import { useMapSettings } from '../hooks/useMapSettings.ts'
import { defaultSurfaceOptions } from '../services/surfaceMap.ts'
import { MapProgress } from './MapProgress.tsx'
import { useEffect, useMemo, useRef, useState } from 'react'
import type { BinaryCloud } from '../hooks/useBinaryCloud.ts'
import { useOctomap } from '../hooks/useOctomap.ts'
import { useNoGo } from '../hooks/useNoGo.ts'
import { CLOUD_HEIGHT_GRADIENT, cloudColorRange } from '../services/cloudHeightColor.ts'
import { Scene3D, type Scene3DHandle } from './Scene3D.tsx'
import styles from './MapView.module.css'

const emptyCloud: BinaryCloud = {
  positions: new Float32Array(0), colors: new Float32Array(0), count: 0, seq: 0,
  protocolVersion: null, frameId: null, epoch: null, stampS: null, sequence: null,
  streamKind: null, connected: false, transport: 'none', lastFrameAt: null, error: null,
}
const hiddenLayers = { grid: false, cloud: false, trail: false, path: false, goal: false,
  robot: false, elevation: false, nativeTraversability: false, localPlanner: false }

interface SavedMapVoxelsProps {
  mapName: string
  contentEpoch?: number | null
}

export function SavedMapVoxels({ mapName, contentEpoch }: SavedMapVoxelsProps) {
  const [showSupport, setShowSupport] = useState(false)
  const settings = useMapSettings()
  const support = useSurfaceMap(mapName, contentEpoch, settings?.surfaceOptions ?? defaultSurfaceOptions, showSupport)
  const state = useOctomap(mapName, contentEpoch, undefined, true)
  const noGo = useNoGo(mapName, contentEpoch)
  const scene = useRef<Scene3DHandle>(null)
  const map = state.status === 'ready' ? state.map : undefined
  const colorRange = useMemo(() => map ? cloudColorRange(map.cells, 4) : null, [map])
  useEffect(() => { if (map) scene.current?.fitMap() }, [map])

  if (!map) return <div className={styles.emptyCanvas} role={state.status === 'error' ? 'alert' : 'status'}>
    <p>{state.status === 'error' ? state.message || '导航体素读取失败' : '正在读取导航体素…'}</p>
  </div>

  return <div className={styles.voxelPreview}>
    <Scene3D ref={scene} navigationMap={map} navigationMapRadius={Infinity} noGoRegions={noGo.map?.regions} supportMap={support.map}
      savedMapEpoch={contentEpoch} cloud={emptyCloud} scanVisible={false} savedMapVisible={false}
      cloudColorMode="height" cloudColorRange={colorRange}
      elevationState={{ status: 'unavailable', message: '' }}
      nativeTraversabilityState={{ status: 'unavailable', message: '' }} sceneGraph={null}
      robotX={0} robotY={0} robotValid={false} followRobot={false} yaw={0}
      trail={[]} path={[]} localPath={[]} layers={hiddenLayers} pointSize={0.04} />
    <div className={styles.voxelToolbar}>
      <span>OctoMap · {Number((map.resolution * 100).toFixed(2))} cm · {(map.cells.length / 4).toLocaleString('zh-CN')} 个占据格</span>
      <label><input type="checkbox" checked={showSupport} onChange={event => setShowSupport(event.currentTarget.checked)} />补全支撑</label>
      <button className={styles.quietButton} onClick={() => scene.current?.fitMap()}>适应整图</button>
    </div>
    {colorRange && <div className={styles.voxelColorScale} aria-label="高度色标">
      <span>{colorRange.min.toFixed(2)} m</span>
      <i style={{ background: CLOUD_HEIGHT_GRADIENT }} aria-hidden="true" />
      <span>{colorRange.max.toFixed(2)} m</span>
    </div>}
    <p className={styles.voxelHint}>显示已保存的占据体素；空白可能是自由或未知空间，不代表可以通行。</p>
    {showSupport && <div className={styles.surfaceLegend}>
      {support.loading && <MapProgress stage="计算多层补面预览" />}
      {support.map && <span>青：实测 {support.map.observed_count.toLocaleString()} 格 · 橙：推断 {support.map.inferred_count.toLocaleString()} 格（{support.map.inferred_area_m2.toFixed(2)} m²）{support.map.sampled ? ' · 采样显示' : ''}</span>}
      {support.error && <span role="alert">{support.error}</span>}
    </div>}
    {noGo.error && <p role="alert" className={styles.voxelHint}>禁行区读取失败：{noGo.error}</p>}
  </div>
}
