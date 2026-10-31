import { forwardRef, useEffect, useImperativeHandle, useState } from 'react'
import { ArrowLeft, ArrowUpRight, Check, MousePointer2, Plus, Scan, Trash2 } from 'lucide-react'
import * as api from '../services/api'
import type { Bounds, StairAnnotation, StairDraft, StairScene, StairTool, XYZ } from '../types/stairs'
import styles from './StairEditor.module.css'

export interface StairEditorHandle {
  setBounds(bounds: Bounds): void
  setPoint(point: XYZ): void
}
interface Props {
  map: string
  epoch: number
  frame: string
  onSceneChange(scene: StairScene): void
  onClose(): void
}

function Coordinate({ label, point, onChange }: { label: string; point?: XYZ; onChange(point: XYZ): void }) {
  return <fieldset className={styles.coordinates} disabled={!point}>
    <legend>{label}</legend>
    {(['x', 'y', 'z'] as const).map(axis => <label key={axis}>
      <span>{axis.toUpperCase()}</span>
      <input aria-label={`${label} ${axis.toUpperCase()}`} type="number" step="0.01"
        key={point?.[axis]} defaultValue={point?.[axis].toFixed(2) ?? ''}
        onBlur={event => {
          if (!point) return
          const n = event.currentTarget.valueAsNumber
          if (Number.isFinite(n)) onChange({ ...point, [axis]: n })
          else event.currentTarget.value = point[axis].toFixed(2)
        }} />
    </label>)}
  </fieldset>
}

export const StairEditor = forwardRef<StairEditorHandle, Props>(function StairEditor(
  { map, epoch, frame, onSceneChange, onClose }, ref,
) {
  const [annotations, setAnnotations] = useState<StairAnnotation[]>([])
  const [draft, setDraft] = useState<StairDraft | null>(null)
  const [id, setId] = useState<string | null>(null)
  const [name, setName] = useState('楼梯')
  const [tool, setTool] = useState<StairTool>(null)
  const [busy, setBusy] = useState(false)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState('')
  const [notice, setNotice] = useState('')
  const [deleteConfirm, setDeleteConfirm] = useState(false)
  const [reload, setReload] = useState(0)
  useEffect(() => {
    let active = true
    api.fetchMapStairs(map).then(result => {
      if (!active) return
      if (result.content_epoch !== epoch) throw new Error('地图已更新，请重新加载地图后标注。')
      setAnnotations(result.stairs)
      setError('')
    }).catch(e => { if (active) setError(e instanceof Error ? e.message : '楼梯标注加载失败') })
      .finally(() => { if (active) setLoading(false) })
    return () => { active = false }
  }, [map, epoch, reload])
  useEffect(() => {
    onSceneChange({ annotations: annotations.filter(a => a.content_epoch === epoch && a.id !== id),
      draft: draft ?? undefined, tool: busy ? null : tool })
  }, [annotations, draft, tool, busy, id, epoch, onSceneChange])
  useImperativeHandle(ref, () => ({
    setBounds(bounds) {
      if (busy || tool !== 'bounds') return
      setDraft(value => ({ ...value, bounds }))
      setTool('lower')
      setError('')
    },
    setPoint(point) {
      if (busy || (tool !== 'lower' && tool !== 'upper')) return
      setDraft(value => ({ ...value, [tool]: point }))
      setTool(tool === 'lower' ? 'upper' : null)
      setError('')
    },
  }), [busy, tool])

  const start = (annotation?: StairAnnotation) => {
    setId(annotation?.id ?? api.makeRequestId('stair'))
    setName(annotation?.name ?? '楼梯')
    const current = annotation?.content_epoch === epoch
    setDraft(annotation && current ? { bounds: annotation.bounds, lower: annotation.lower, upper: annotation.upper } : {})
    setTool(annotation && current ? null : 'bounds')
    setNotice(annotation && !current ? '地图已变化，请重新框选范围和两个入口。' : '')
    setError('')
    setDeleteConfirm(false)
  }
  const discard = () => { setDraft(null); setId(null); setTool(null); setDeleteConfirm(false); setError('') }
  const save = async () => {
    if (!id || !draft?.bounds || !draft.lower || !draft.upper) return
    if (!name.trim()) { setError('为这段楼梯填写名称。'); return }
    if (draft.lower.z >= draft.upper.z) { setError('上入口应高于下入口，请检查两个平台位置。'); return }
    if (['x', 'y', 'z'].some(axis => draft.bounds!.min[axis as keyof XYZ] >= draft.bounds!.max[axis as keyof XYZ])) {
      setError('范围上限应大于下限。'); return
    }
    setBusy(true)
    setError('')
    try {
      const result = await api.saveMapStair(map, { id, name: name.trim(),
        content_epoch: epoch, frame_id: frame, bounds: draft.bounds, lower: draft.lower, upper: draft.upper })
      setAnnotations(result.stairs)
      discard()
      const geometry = result.stairs.find(a => a.id === id)?.geometry
      setNotice(geometry?.recognized ? '已保存，已识别梯级与上下平台。' : '已保存标注，尚未确认完整梯级与平台。')
    } catch (e) { setError(e instanceof Error ? e.message : '保存失败，标注仍保留在这里。') }
    finally { setBusy(false) }
  }
  const remove = async () => {
    if (!id) return
    setBusy(true)
    try {
      const result = await api.deleteMapStair(map, id, epoch)
      setAnnotations(result.stairs)
      discard()
      setNotice('已删除标注')
    } catch (e) { setError(e instanceof Error ? e.message : '删除失败') }
    finally { setBusy(false) }
  }
  const instructions = tool === 'bounds' ? '在点云上拖框选择这段楼梯。选完后可调整三维范围。'
    : tool === 'lower' ? '点击楼下平台表面，给进入前的准备留出空间。'
      : tool === 'upper' ? '点击楼上平台表面，作为离开楼梯的位置。'
        : '拖动地图检查范围与两个入口，再保存。'

  return <section className={styles.editor} aria-label="楼梯标注">
    <header><button type="button" onClick={onClose} disabled={busy} aria-label="返回操作"><ArrowLeft size={18} /></button>
      <strong>楼梯标注</strong><span>{map}</span></header>
    {error && <p className={styles.error} role="alert">{error}{!draft && <button onClick={() => setReload(v => v + 1)}>重试</button>}</p>}
    {notice && <p className={styles.notice} role="status"><Check size={14} />{notice}</p>}
    {!draft ? <>
      <button type="button" className={styles.add} disabled={loading || !!error} onClick={() => start()}><Plus size={16} />标注楼梯</button>
      {loading ? <p>正在加载标注…</p> : annotations.length === 0 ? <p className={styles.hint}>框选一段楼梯，再点选上下平台入口。</p>
        : <ul className={styles.list}>{annotations.map(a => <li key={a.id}>
          <button onClick={() => start(a)}><ArrowUpRight size={18} /><span>{a.name}
            <small>{a.content_epoch !== epoch ? '地图已变化 · 需要重新对齐' : `${a.lower.z.toFixed(2)} → ${a.upper.z.toFixed(2)} m · ${a.geometry?.recognized ? '梯级已识别' : '待确认几何'}`}</small></span></button>
        </li>)}</ul>}
      <p className={styles.hint}>保存时根据原始点云识别梯级与平台。几何识别通过后，还需验证机器人的楼梯通行能力。</p>
    </> : <>
      <label className={styles.name}>名称<input aria-label="楼梯名称" value={name} maxLength={100} disabled={busy} onChange={e => setName(e.target.value)} /></label>
      <div className={styles.steps} role="group" aria-label="标注步骤">
        {(['bounds', 'lower', 'upper'] as const).map((value, index) => <button key={value} type="button" disabled={busy}
          aria-pressed={tool === value} onClick={() => setTool(tool === value ? null : value)}>
          {index + 1} {value === 'bounds' ? '框范围' : value === 'lower' ? '下入口' : '上入口'}
        </button>)}
      </div>
      <p className={styles.instruction} role="status">{tool ? <Scan size={16} /> : <MousePointer2 size={16} />}{instructions}</p>
      {tool && <button type="button" className={styles.link} onClick={() => setTool(null)}>暂时转动视角</button>}
      <div className={styles.summary}>
        <span>范围 {draft.bounds ? '已框选' : '待框选'}</span>
        <span>下入口 {draft.lower ? `${draft.lower.z.toFixed(2)} m` : '待选择'}</span>
        <span>上入口 {draft.upper ? `${draft.upper.z.toFixed(2)} m` : '待选择'}</span>
      </div>
      <details className={styles.details}>
        <summary>调整范围与入口</summary>
        <p className={styles.hint}>入口记录平台表面坐标，单位 m。</p>
        <Coordinate label="范围下限" point={draft.bounds?.min} onChange={p => setDraft({ ...draft, bounds: { ...draft.bounds!, min: p } })} />
        <Coordinate label="范围上限" point={draft.bounds?.max} onChange={p => setDraft({ ...draft, bounds: { ...draft.bounds!, max: p } })} />
        <Coordinate label="下入口" point={draft.lower} onChange={p => setDraft({ ...draft, lower: p })} />
        <Coordinate label="上入口" point={draft.upper} onChange={p => setDraft({ ...draft, upper: p })} />
      </details>
      <div className={styles.actions}>
        <button type="button" className={styles.primary} disabled={busy || !draft.bounds || !draft.lower || !draft.upper} onClick={save}>{busy ? '正在保存…' : '保存标注'}</button>
        <button type="button" disabled={busy} onClick={discard}>取消</button>
      </div>
      {id && annotations.some(annotation => annotation.id === id) && <div className={styles.delete}>
        {deleteConfirm ? <><span>删除这段标注？</span><button disabled={busy} onClick={remove}>确认删除</button><button onClick={() => setDeleteConfirm(false)}>保留</button></>
          : <button disabled={busy} onClick={() => setDeleteConfirm(true)}><Trash2 size={14} />删除标注</button>}
      </div>}
    </>}
  </section>
})
