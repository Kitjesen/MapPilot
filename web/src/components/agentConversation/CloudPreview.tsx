import { lazy, Suspense, useCallback, useEffect, useRef, useState } from 'react'
import { Box, Camera, ChevronUp, Download, Map, RotateCcw } from 'lucide-react'
import { previewEditedMap } from '../../services/api.ts'
import { readMapEditRecipe } from '../../services/mapEdit.ts'
import type { MapEditPreview } from '../../types/mapEdit.ts'
import type { MapGallerySceneHandle } from '../MapGalleryScene.tsx'
import { useMapSettings } from '../../hooks/useMapSettings.ts'
import styles from './CloudPreview.module.css'

const Scene = lazy(() => import('../MapGalleryScene.tsx').then(module => ({ default: module.MapGalleryScene })))
type Loaded = ReturnType<typeof readMapEditRecipe> & { preview: MapEditPreview }

export function CloudPreview({ result }: { result: Record<string, unknown> }) {
  const settings = useMapSettings()
  const [loaded, setLoaded] = useState<Loaded | null>(null)
  const [open, setOpen] = useState(false)
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState('')
  const [image, setImage] = useState('')
  const scene = useRef<MapGallerySceneHandle | null>(null)
  const request = useRef<AbortController | null>(null)
  const sceneReady = useCallback((handle: MapGallerySceneHandle | null) => { scene.current = handle }, [])
  useEffect(() => () => request.current?.abort(), [])

  async function show() {
    if (loaded) { setOpen(true); return }
    if (request.current) return
    const abort = new AbortController()
    request.current = abort
    setBusy(true); setError('')
    try {
      const recipe = readMapEditRecipe(result)
      const preview = await previewEditedMap(recipe.source, recipe.operations, abort.signal)
      if (abort.signal.aborted) return
      setLoaded({ ...recipe, preview }); setOpen(true)
    } catch (cause) {
      if (!abort.signal.aborted) setError(cause instanceof Error ? cause.message : String(cause))
    } finally {
      request.current = null
      if (!abort.signal.aborted) setBusy(false)
    }
  }

  function capture(close = false) {
    try {
      const snapshot = scene.current?.snapshot()
      if (snapshot) setImage(snapshot)
      if (close) setOpen(false)
    } catch { setError('截图失败，可继续查看三维预览。') }
  }

  const preview = loaded?.preview
  const stats = preview ?? result
  const matching = loaded && settings?.map?.name === loaded.source.name && settings.map.contentEpoch === loaded.source.contentEpoch
  return <section className={styles.card} aria-label="点云编辑预览">
    <header><Box size={14} aria-hidden="true" /><strong>点云预览</strong><span>处理后 · 尚未保存</span></header>
    {loaded && <p className={styles.identity}>{loaded.source.name} · 版本 {loaded.source.contentEpoch} · {loaded.source.frameId}</p>}
    <p className={styles.stats}>
      {typeof stats.point_count === 'number' && <span>{stats.point_count.toLocaleString()} 点</span>}
      {typeof stats.removed_points === 'number' && <span>移除 {stats.removed_points.toLocaleString()}</span>}
      {typeof stats.added_points === 'number' && <span>新增 {stats.added_points.toLocaleString()}</span>}
      {stats.preview_sample === true && <span>抽样显示</span>}
    </p>
    {busy && <p role="status">正在读取三维预览…</p>}
    {error && <p className={styles.error} role="alert">{error}</p>}
    {open && loaded && preview && <>
      {preview.preview_points.length ? <div className={styles.viewer}>
        <Suspense fallback={<p role="status">正在加载三维视窗…</p>}>
          <Scene name={`${loaded.source.name} 点云预览`} points={preview.preview_points}
            voxelScale={1} pointSize={2} opacity={1} ceiling={null} grid={false} gridStep={1} onReady={sceneReady} />
        </Suspense>
      </div> : <p className={styles.empty}>处理后的点云为空。</p>}
      <p className={styles.hint}>按高度着色 · 拖动旋转，滚轮缩放，Home 复位</p>
    </>}
    {!open && image && <button type="button" className={styles.thumbnail} onClick={() => void show()} aria-label="展开三维点云">
      <img src={image} alt="本次点云预览的视角截图" /><span><Box size={14} />继续三维查看</span>
    </button>}
    <footer>
      {!open ? <button type="button" disabled={busy} onClick={() => void show()}><Box size={13} />{image ? '三维查看' : '查看三维'}</button>
        : <>
          <button type="button" onClick={() => capture(true)}><ChevronUp size={13} />收起三维</button>
          {preview?.preview_points.length !== 0 && <button type="button" onClick={() => capture()}><Camera size={13} />截图</button>}
          {preview?.preview_points.length !== 0 && <button type="button" onClick={() => scene.current?.resetView()}><RotateCcw size={13} />复位</button>}
        </>}
      {image && <a href={image} download="point-cloud-preview.png"><Download size={13} />下载截图</a>}
      {loaded && settings && <button type="button" disabled={!matching} title={!matching ? '请先打开对应版本的地图' : undefined}
        onClick={() => settings.setAgentPreview({ mapId: loaded.source.name, contentEpoch: loaded.source.contentEpoch,
          scene: { selecting: false, selection: null, view: 'draft', operations: loaded.operations, showDeleted: true, preview: loaded.preview } })}>
        <Map size={13} />在主地图查看</button>}
    </footer>
  </section>
}
