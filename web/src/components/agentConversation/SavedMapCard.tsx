import { lazy, Suspense, useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { Box, Camera, ChevronUp, Download, RotateCcw } from 'lucide-react'
import { fetchMapThumbnail, type MapThumbnailData } from '../../services/mapThumbnail.ts'
import type { MapGallerySceneHandle } from '../MapGalleryScene.tsx'
import styles from './CloudPreview.module.css'

const Scene = lazy(() => import('../MapGalleryScene.tsx').then(module => ({ default: module.MapGalleryScene })))

export function SavedMapCard({ name, epoch, autoOpen = true }: { name: string; epoch: number; autoOpen?: boolean }) {
  const root = useRef<HTMLElement>(null)
  const scene = useRef<MapGallerySceneHandle | null>(null)
  const [visible, setVisible] = useState(false)
  const [open, setOpen] = useState(autoOpen)
  const [data, setData] = useState<MapThumbnailData | null>(null)
  const [error, setError] = useState('')
  const [retry, setRetry] = useState(0)
  const [image, setImage] = useState('')
  const points = useMemo(() => data?.points.flat() ?? [], [data])
  const ready = useCallback((handle: MapGallerySceneHandle | null) => { scene.current = handle }, [])
  useEffect(() => {
    const observer = new IntersectionObserver(([entry]) => setVisible(entry.isIntersecting))
    if (root.current) observer.observe(root.current)
    return () => observer.disconnect()
  }, [])
  useEffect(() => {
    if (!open || !visible || data) return
    const abort = new AbortController()
    void fetchMapThumbnail(name, epoch, abort.signal).then(value => {
      if (!abort.signal.aborted) { setError(''); setData(value) }
    }).catch(cause => { if (!abort.signal.aborted) setError(cause instanceof Error ? cause.message : String(cause)) })
    return () => abort.abort()
  }, [name, epoch, open, visible, data, retry])
  function capture(close = false) {
    try { const shot = scene.current?.snapshot(); if (shot) setImage(shot) }
    catch { setError('截图失败，可继续操作地图。') }
    if (close) setOpen(false)
  }
  return <section ref={root} className={styles.card} aria-label={`三维地图 ${name}`}>
    <header><Box size={14} /><strong>{name}</strong><span>已保存 · 只读</span></header>
    <p className={styles.stats}><span>版本 {epoch}</span>{data && <>
      <span>{data.count.toLocaleString()} / {data.total_points.toLocaleString()} 点</span>
      <span>{data.frame_id || '坐标系未记录'}</span>{data.count < data.total_points && <span>抽样预览</span>}
    </>}</p>
    {open && !data && !error && <p role="status">正在读取地图…</p>}
    {error && <p className={styles.error} role="alert">{error}</p>}
    {open && data && <div className={styles.viewer}>
      {!points.length ? <p className={styles.empty}>这张地图没有可显示的点。</p> : visible && <Suspense fallback={<p role="status">正在加载三维视窗…</p>}>
        <Scene name={name} points={points} voxelScale={1} pointSize={2} opacity={1} ceiling={null} grid={false} gridStep={1} onReady={ready} />
      </Suspense>}
    </div>}
    {!open && image && <button className={styles.thumbnail} type="button" onClick={() => setOpen(true)} aria-label="展开已保存地图">
      <img src={image} alt={`${name} 的三维视角`} /></button>}
    <footer>
      {!open ? <button type="button" onClick={() => { setError(''); setOpen(true) }}><Box size={13} />查看三维</button>
        : <><button type="button" onClick={() => capture(true)}><ChevronUp size={13} />收起</button>
          <button type="button" disabled={!data || !points.length} onClick={() => scene.current?.resetView()}><RotateCcw size={13} />复位</button>
          <button type="button" disabled={!data || !points.length} onClick={() => capture()}><Camera size={13} />截图</button></>}
      {error && !data && <button type="button" onClick={() => { setError(''); setRetry(value => value + 1) }}>重试</button>}
      {image && <a href={image} download={`${name}-preview.png`}><Download size={13} />下载截图</a>}
    </footer>
    {open && data && <p className={styles.hint}>拖动旋转 · 滚轮缩放 · 右键平移</p>}
  </section>
}
