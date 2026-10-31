import { forwardRef, useCallback, useEffect, useImperativeHandle, useMemo, useRef, useState } from 'react'
import { createPortal } from 'react-dom'
import { ArrowRight, BoxSelect, ChevronDown, MoreHorizontal, Redo2, SlidersHorizontal, Trash2, Undo2, X } from 'lucide-react'
import type { Bounds } from '../types/stairs.ts'
import type { MapEditOperation, MapEditOperationKind, MapEditScene, MapEditSource, MapEditView } from '../types/mapEdit.ts'
import { MapEditSaveUnconfirmed, applyMapEditOperations, mapEditCloudBounds, mapEditNameError, mapEditSelectionImpact, suggestedEditedMapName, validMapEditBounds } from '../services/mapEdit.ts'
import { readMapDraft, removeMapDraft, writeMapDraft } from '../services/mapDraft.ts'
import type { MapDraftResult, MapEditDraft } from '../services/mapDraft.ts'
import * as api from '../services/api.ts'
import styles from './MapEditor.module.css'

export interface MapEditorHandle { setBounds(bounds: Bounds): void }
interface Props {
  source: MapEditSource
  points: readonly number[]
  anchor: { x: number; y: number } | null
  footerTarget: HTMLElement | null
  cameraExpanded?: boolean
  onExitGuardChange?: (guard: ((proceed: () => void) => void) | null) => void
  onSceneChange(scene: MapEditScene): void
  onView(view: MapEditView): void
  onClose(): void
  onOpenMap(name: string): void
  onMapsChanged(): void
}

const copyBounds = (bounds: Bounds): Bounds => ({ min: { ...bounds.min }, max: { ...bounds.max } })
const pointCount = new Intl.NumberFormat('zh-CN')
const pointPercent = new Intl.NumberFormat('zh-CN', { style: 'percent', maximumFractionDigits: 1 })

export const MapEditor = forwardRef<MapEditorHandle, Props>(function MapEditor(
  { source, points, anchor, footerTarget, cameraExpanded = false, onExitGuardChange, onSceneChange, onView, onClose, onOpenMap, onMapsChanged }, ref,
) {
  const { name: sourceName, contentEpoch, frameId } = source
  const [storedDraft] = useState(() => readMapDraft(source))
  const [recovery, setRecovery] = useState<MapDraftResult | null>(() => storedDraft?.kind === 'unconfirmed' ? null : storedDraft)
  const [pendingSave, setPendingSave] = useState<MapEditDraft | null>(() => storedDraft?.kind === 'unconfirmed' ? storedDraft.draft : null)
  const [history, setHistory] = useState<{ edits: MapEditOperation[]; cursor: number }>({ edits: [], cursor: 0 })
  const operations = useMemo(() => history.edits.slice(0, history.cursor), [history])
  const [selection, setSelection] = useState<Bounds | null>(null)
  const [selecting, setSelecting] = useState(false)
  const [view, setView] = useState<'original' | 'draft'>('draft')
  const [showDeleted, setShowDeleted] = useState(false)
  const [dialog, setDialog] = useState<'save' | 'discard' | null>(null)
  const [name, setName] = useState(() => pendingSave?.name ?? suggestedEditedMapName(source.name))
  const [error, setError] = useState('')
  const [phase, setPhase] = useState<'editing' | 'saving' | 'unconfirmed' | 'building' | 'build_failed' | 'saved'>(() => pendingSave ? 'unconfirmed' : 'editing')
  const [saved, setSaved] = useState<{
    name: string; options?: Record<string, unknown>; pointCount?: number; removedPoints?: number; previewSample: boolean
  } | null>(null)
  const [viewport, setViewport] = useState({ width: 1000, height: 700, bottom: 688 })
  const [selectionHeight, setSelectionHeight] = useState(300)
  const [recordOpen, setRecordOpen] = useState(false)
  const rootRef = useRef<HTMLDivElement>(null)
  const selectionRef = useRef<HTMLElement>(null)
  const dialogRef = useRef<HTMLDialogElement>(null)
  const nameRef = useRef<HTMLInputElement>(null)
  const closeRef = useRef<HTMLButtonElement>(null)
  const saveRef = useRef<HTMLButtonElement>(null)
  const selectRef = useRef<HTMLButtonElement>(null)
  const opener = useRef<HTMLElement | null>(null)
  const pendingLeave = useRef<(() => void) | null>(null)
  const alive = useRef(true)
  const busy = phase === 'saving' || phase === 'building'
  const locked = busy || !!pendingSave || !!recovery || !!saved
  const dirty = !saved && (history.edits.length > 0 || !!pendingSave)
  const cloudBounds = useMemo(() => mapEditCloudBounds(points), [points])
  const draftPoints = useMemo(() => applyMapEditOperations(points, operations), [points, operations])
  const selectionImpact = useMemo(() => selection ? mapEditSelectionImpact(draftPoints, selection) : null, [draftPoints, selection])
  const fullHeight = selection && cloudBounds && selection.min.z <= cloudBounds.min.z && selection.max.z >= cloudBounds.max.z
  const savedCounts = saved ? [
    typeof saved.pointCount === 'number' ? `保留 ${pointCount.format(saved.pointCount)} 点` : null,
    typeof saved.removedPoints === 'number' ? `删除 ${pointCount.format(saved.removedPoints)} 点` : null,
  ].filter(Boolean).join(' · ') : ''

  useEffect(() => { alive.current = true; return () => { alive.current = false } }, [])
  useEffect(() => { selectRef.current?.focus() }, [])
  useEffect(() => {
    if (recovery || pendingSave || saved || history.edits.length === 0) return
    writeMapDraft({ source: { name: sourceName, contentEpoch, frameId }, name, history, status: 'editing' })
  }, [sourceName, contentEpoch, frameId, name, history, recovery, pendingSave, saved])
  useEffect(() => {
    const element = rootRef.current
    if (!element) return
    const observer = new ResizeObserver(() => {
      const canvas = element.getBoundingClientRect()
      const footer = footerTarget?.parentElement?.getBoundingClientRect()
      const bottom = canvas.height - (canvas.width < 760 ? 74 : 12)
      setViewport({ width: canvas.width, height: canvas.height, bottom: footer?.height
        ? Math.min(bottom, footer.top - canvas.top - 12) : bottom })
    })
    observer.observe(element)
    if (footerTarget) observer.observe(footerTarget)
    if (footerTarget?.parentElement) observer.observe(footerTarget.parentElement)
    return () => observer.disconnect()
  }, [footerTarget])
  useEffect(() => {
    onSceneChange({ selecting: !locked && selecting, selection, operations, view, showDeleted })
  }, [selecting, selection, operations, view, showDeleted, locked, onSceneChange])
  useEffect(() => {
    const element = selectionRef.current
    if (!selection || !element) return
    const observer = new ResizeObserver(() => setSelectionHeight(element.scrollHeight + 2))
    observer.observe(element)
    return () => observer.disconnect()
  }, [selection])
  useEffect(() => {
    const element = dialogRef.current
    if (!element) return
    if (dialog) {
      opener.current = dialog === 'save' ? saveRef.current
        : document.activeElement instanceof HTMLElement ? document.activeElement : null
      element.showModal()
      if (dialog === 'save') { nameRef.current?.focus(); nameRef.current?.select() }
    } else if (element.open) {
      element.close()
      opener.current?.focus()
    }
  }, [dialog])
  useImperativeHandle(ref, () => ({ setBounds(bounds) {
    if (locked) return
    setSelection(copyBounds(bounds)); setSelecting(false); setError(''); setView('draft')
  } }), [locked])

  const requestClose = useCallback(() => {
    if (busy) return
    if (dirty) { pendingLeave.current = null; setDialog('discard') }
    else onClose()
  }, [busy, dirty, onClose])
  useEffect(() => {
    onExitGuardChange?.(proceed => {
      if (busy) { setError('正在处理地图，请等待结果后切换页面。'); return }
      if (dirty) { pendingLeave.current = proceed; setDialog('discard') }
      else proceed()
    })
    return () => onExitGuardChange?.(null)
  }, [busy, dirty, onExitGuardChange])
  const undo = useCallback(() => {
    if (locked) return
    setHistory(value => ({ ...value, cursor: Math.max(0, value.cursor - 1) }))
    setSelection(null); setSelecting(false); setError('')
  }, [locked])
  const redo = useCallback(() => {
    if (locked) return
    setHistory(value => ({ ...value, cursor: Math.min(value.edits.length, value.cursor + 1) }))
    setSelection(null); setSelecting(false); setError('')
  }, [locked])
  useEffect(() => {
    const handleKey = (event: KeyboardEvent) => {
      if (event.defaultPrevented || dialog || cameraExpanded || document.fullscreenElement || (event.target instanceof HTMLElement
        && event.target.closest('input, textarea, select, [contenteditable="true"]'))) return
      if (event.key === 'Escape') {
        event.preventDefault(); event.stopImmediatePropagation()
        const disclosure = rootRef.current?.querySelector<HTMLDetailsElement>('details[open]')
          ?? footerTarget?.querySelector<HTMLDetailsElement>('details[open]')
        if (disclosure) { disclosure.open = false; disclosure.querySelector<HTMLElement>('summary')?.focus() }
        else if (selecting || selection) { setSelecting(false); setSelection(null); selectRef.current?.focus() }
        else requestClose()
      } else if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === 'z') {
        event.preventDefault(); if (event.shiftKey) redo(); else undo()
      }
    }
    window.addEventListener('keydown', handleKey, true)
    return () => window.removeEventListener('keydown', handleKey, true)
  }, [dialog, selecting, selection, requestClose, redo, undo, footerTarget, cameraExpanded])
  useEffect(() => {
    if (!dirty && !busy) return
    const guard = (event: BeforeUnloadEvent) => { event.preventDefault(); event.returnValue = '' }
    window.addEventListener('beforeunload', guard)
    return () => window.removeEventListener('beforeunload', guard)
  }, [dirty, busy])

  const startSelection = () => {
    setSelecting(value => !value); setSelection(null); setView('draft'); setError('')
  }
  const numericSelection = () => {
    if (!cloudBounds) return
    setSelection(copyBounds(cloudBounds)); setSelecting(false); setView('draft'); setError('')
  }
  const apply = (operation: MapEditOperationKind) => {
    if (!selection || !validMapEditBounds(selection) || locked) return
    const edit = { operation, bounds: copyBounds(selection) }
    const next = [...operations, edit]
    if (applyMapEditOperations(draftPoints, [edit]).length === 0) {
      setError('该操作会清空预览中的点云，请缩小范围。'); return
    }
    setHistory({ edits: next, cursor: next.length }); setSelection(null); setSelecting(false); setError(''); setView('draft')
    selectRef.current?.focus()
  }
  const updateCoordinate = (edge: 'min' | 'max', axis: 'x' | 'y' | 'z', input: HTMLInputElement) => {
    if (!selection) return
    const value = input.valueAsNumber
    const next = copyBounds(selection); next[edge][axis] = value
    if (validMapEditBounds(next)) { setSelection(next); setError('') }
    else { input.value = selection[edge][axis].toFixed(2); setError('范围上限须大于下限，数值已恢复。') }
  }
  const rebuild = async (target: { name: string; options?: Record<string, unknown> }) => {
    setPhase('building'); setError('')
    try {
      const result = await api.buildMapOctomap(target.name, target.options)
      if (!(result.ok || result.success)) throw new Error(result.message || '重建失败')
      if (!alive.current) return
      setPhase('saved'); onMapsChanged()
    } catch (reason) {
      if (!alive.current) return
      setPhase('build_failed'); setError(reason instanceof Error ? reason.message : '导航数据重建失败')
    }
  }
  const save = async () => {
    const request: MapEditDraft = pendingSave ?? { source, name: name.trim(), history, status: 'unconfirmed' }
    const invalid = mapEditNameError(request.name, request.source.name)
    if (invalid) { setError(invalid); return }
    writeMapDraft(request)
    setPendingSave(request)
    setDialog(null); setSelecting(false); setSelection(null); setPhase('saving'); setError('')
    try {
      const result = await api.saveEditedMap(request.source, request.name, request.history.edits.slice(0, request.history.cursor))
      if (!(result.ok || result.success)) throw new Error(result.message || '另存失败')
      removeMapDraft(request.source)
      if (!alive.current) return
      const target = { name: request.name, options: result.build_options,
        pointCount: result.point_count, removedPoints: result.removed_points,
        previewSample: result.point_count_scope === 'preview_sample' }
      setSaved(target); setPendingSave(null); onMapsChanged()
      await rebuild(target)
    } catch (reason) {
      if (!alive.current) return
      setError(reason instanceof Error ? reason.message : '另存失败，草稿已保留')
      if (reason instanceof MapEditSaveUnconfirmed) { setPhase('unconfirmed'); return }
      const draft: MapEditDraft = { ...request, status: 'editing' }
      writeMapDraft(draft)
      setPendingSave(null); setPhase('editing')
      if (request.source.contentEpoch !== source.contentEpoch || request.source.frameId !== source.frameId) {
        setHistory({ edits: [], cursor: 0 }); setRecovery({ kind: 'stale', draft })
      } else {
        setHistory(request.history); setName(request.name); setDialog('save')
      }
    }
  }
  const position = {
    left: Math.max(12, Math.min((anchor?.x ?? viewport.width / 2) + 18, viewport.width - 356)),
    top: Math.max(viewport.width < 760 ? Math.max(86, Math.min(245, viewport.bottom - 240)) : 86,
      Math.min((anchor?.y ?? 100) + 18, viewport.bottom - selectionHeight)),
  }
  const coordinates = (axis: 'x' | 'y' | 'z') => (['min', 'max'] as const).map(edge => <label key={`${edge}-${axis}`}>
    <span>{axis.toUpperCase()} {edge === 'min' ? '下限' : '上限'}<small>m</small></span>
    <input type="number" aria-label={`选区 ${axis.toUpperCase()} ${edge === 'min' ? '下限' : '上限'}`}
      key={`${edge}-${axis}-${selection?.[edge][axis]}`} defaultValue={selection?.[edge][axis].toFixed(2)} step="0.05"
      onBlur={event => updateCoordinate(edge, axis, event.currentTarget)}
      onKeyDown={event => {
        if (event.key === 'Enter') { event.preventDefault(); event.currentTarget.blur() }
        if (event.key === 'Escape' && selection) { event.preventDefault(); event.stopPropagation(); event.currentTarget.value = selection[edge][axis].toFixed(2); event.currentTarget.blur() }
      }} />
  </label>)
  const menuItems = <>
    <button type="button" disabled={locked} onClick={numericSelection}><SlidersHorizontal size={16} />输入选区范围</button>
    <label><input type="checkbox" checked={showDeleted} onChange={event => setShowDeleted(event.target.checked)} />显示删除点</label>
    <button type="button" disabled={locked || history.cursor >= history.edits.length} onClick={redo}><Redo2 size={16} />重做</button>
    <button type="button" onClick={() => setRecordOpen(value => !value)}>修改记录<ChevronDown size={15} /></button>
    {recordOpen && <ol>{operations.length ? operations.map((op, index) => <li key={index}>
      <button type="button" onClick={() => { setSelection(copyBounds(op.bounds)); setSelecting(false); onView('three') }}>
        {index + 1}. {op.operation === 'delete_inside' ? '删除框内' : '裁掉框外'}</button>
    </li>) : <li>尚无修改</li>}</ol>}
    <p>{source.name}<br />地图版本 {source.contentEpoch} · {source.frameId}<br />已加载完整点云，修改另存为新地图。</p>
  </>
  const footer = (history.edits.length > 0 || recovery || pendingSave || busy || saved || error) && <div className={styles.footerStack}>
    {error && dialog !== 'save' && <p role="alert" className={styles.error}>{error}</p>}
    <div className={styles.bar} aria-label="地图编辑草稿" aria-busy={busy}>
      <div className={styles.barSummary}>
        <strong role="status">{recovery ? recovery.kind === 'ready' ? '发现未保存草稿' : '原地图已更新' : phase === 'saving' ? '正在保存点云' : phase === 'building' ? '正在重建导航数据'
          : phase === 'unconfirmed' ? '保存结果待确认' : phase === 'build_failed' ? '点云已保存 · 重建失败' : phase === 'saved' ? '新地图已保存' : operations.length ? '未保存修改' : '尚无修改'}</strong>
        {recovery && <span className={styles.savedCounts}>{recovery.kind === 'ready'
          ? `${recovery.draft.history.cursor} 步修改${recovery.draft.history.edits.length > recovery.draft.history.cursor
            ? ` · ${recovery.draft.history.edits.length - recovery.draft.history.cursor} 步可重做` : ''} · ${recovery.draft.name}`
          : '旧草稿不能应用到当前地图'}</span>}
        {phase !== 'editing' && <span className={styles.targetName} title={saved?.name ?? name.trim()}>{saved?.name ?? name.trim()}</span>}
        {savedCounts && <span className={styles.savedCounts}>{saved?.previewSample ? '预览样本统计：' : '完整点云：'}{savedCounts}</span>}
      </div>
      {recovery ? <div className={styles.resultActions}>
        <button type="button" className={styles.secondary} onClick={() => { removeMapDraft(source); setRecovery(null); setError('') }}>放弃旧草稿</button>
        {recovery.kind === 'ready' && <button type="button" className={styles.primary} onClick={() => {
          setHistory(recovery.draft.history); setName(recovery.draft.name); setRecovery(null); setError(''); setView('draft')
        }}>恢复草稿</button>}
      </div> : busy ? null : phase === 'saved' ? <div className={styles.resultActions}>
        <button type="button" className={styles.primary} title="查看新地图，不切换当前活动地图" onClick={() => saved && onOpenMap(saved.name)}>查看新地图<ArrowRight size={17} /></button></div>
        : phase === 'unconfirmed' ? <button type="button" className={styles.primary} onClick={() => void save()}>确认保存结果</button>
          : phase === 'build_failed' && saved ? <div className={styles.resultActions}>
            <button type="button" className={styles.secondary} title="仅查看已保存点云，导航数据尚未就绪" onClick={() => onOpenMap(saved.name)}>查看已保存点云</button>
            <button type="button" className={styles.primary} onClick={() => void rebuild(saved)}>重试重建</button>
          </div>
          : <>
            <button type="button" className={styles.undo} aria-label="撤销上一步" title="撤销上一步（Ctrl+Z）" disabled={locked || history.cursor === 0} onClick={undo}><Undo2 size={19} /><span>撤销</span></button>
            <details className={styles.more}><summary aria-label="更多编辑操作" title="更多编辑操作"><MoreHorizontal size={21} /></summary><div className={styles.menu}>{menuItems}</div></details>
            <button type="button" ref={saveRef} className={styles.primary} aria-label="另存新地图" disabled={busy || operations.length === 0 || !!saved}
              onClick={() => { setError(''); setDialog('save') }}><span className={styles.saveLong}>另存新地图</span><span className={styles.saveShort}>另存</span><ArrowRight size={18} /></button>
          </>}
    </div>
  </div>

  return <div className={styles.editor} ref={rootRef} data-map-editor>
    <div className={styles.toolbar} role="group" aria-label="点云整理工具">
      <strong className={styles.mode}>点云整理</strong>
      <button type="button" ref={selectRef} aria-label="框选范围" aria-pressed={selecting} title="框选三维范围" disabled={locked} onClick={startSelection}><BoxSelect size={20} /><span className={styles.toolLabel}>框选</span></button>
      <button type="button" className={styles.numeric} aria-label="输入选区范围" title="输入选区范围" disabled={locked} onClick={numericSelection}><SlidersHorizontal size={18} /></button>
      <div className={styles.compare} role="group" aria-label="原图与草稿对比">
        {(['original', 'draft'] as const).map(value => <button type="button" key={value} aria-pressed={view === value}
          onClick={() => { setView(value); if (value === 'original') { setSelecting(false); setSelection(null) } }}>{value === 'original' ? '原图' : '草稿'}</button>)}
      </div>
      <details className={`${styles.more} ${styles.mobileMore}`}><summary aria-label="编辑视图与操作"><MoreHorizontal size={20} /></summary><div className={styles.menu}>
        <button type="button" onClick={() => { setView('original'); setSelecting(false); setSelection(null) }}>查看原图</button>
        <button type="button" onClick={() => setView('draft')}>查看草稿</button>{menuItems}
      </div></details>
      <button type="button" ref={closeRef} aria-label="退出地图编辑" title="退出编辑" disabled={busy} onClick={requestClose}><X size={19} /></button>
    </div>
    {selecting && <p className={styles.hint} role="status">在俯视图拖动框选 · Esc 取消</p>}
    {selection && <section ref={selectionRef} className={styles.selection}
      style={{ ...position, maxHeight: Math.max(120, viewport.bottom - position.top) }} aria-label="调整选区范围">
      <header><strong>选区高度</strong><span>地图 Z</span><button type="button" aria-label="取消本次选区" onClick={() => setSelection(null)}><X size={17} /></button></header>
      {selectionImpact && <div className={styles.selectionImpact} aria-label="选区预览统计">
        <p><span>当前点云</span><strong>选中 {pointCount.format(selectionImpact.selected)} 点 · {pointPercent.format(selectionImpact.total ? selectionImpact.selected / selectionImpact.total : 0)}</strong></p>
        {fullHeight && <p className={styles.heightHint}>覆盖整段高度 · 请留意同位置的地面</p>}
      </div>}
      <div className={styles.coordinates}>{coordinates('z')}</div>
      <div className={styles.views} role="group" aria-label="检查选区视角">{(['top', 'front', 'side', 'three'] as const).map(value =>
        <button key={value} type="button" onClick={() => onView(value)}>{({ top: '俯视', front: '正视', side: '侧视', three: '三维' })[value]}</button>)}</div>
      <details className={styles.bounds}><summary>调整平面范围</summary><div className={styles.coordinates}>{coordinates('x')}{coordinates('y')}</div></details>
      {error && <p role="alert" className={styles.error}>{error}</p>}
      <div className={styles.selectionActions}><button type="button" className={styles.delete} disabled={busy || !!saved || !selectionImpact?.selected} onClick={() => apply('delete_inside')}><Trash2 size={16} />删除选中</button>
        <details className={styles.more}><summary aria-label="更多选区操作"><MoreHorizontal size={20} /></summary><div className={styles.menu}><button type="button" disabled={busy || !!saved} onClick={() => apply('keep_inside')}>裁掉框外，仅保留选区</button></div></details>
      </div>
    </section>}
    {footerTarget ? createPortal(footer, footerTarget) : footer && <div className={styles.fallbackFooter}>{footer}</div>}
    <dialog className={`${styles.dialog} ${dialog === 'save' ? styles.saveDialog : ''}`} ref={dialogRef} aria-labelledby="map-edit-dialog-title"
      onKeyDown={event => { if (event.key === 'Escape') event.stopPropagation() }}
      onCancel={event => { event.preventDefault(); if (!busy) { pendingLeave.current = null; setDialog(null) } }}>
      {dialog === 'save' ? <form onSubmit={event => { event.preventDefault(); if (!busy) void save() }}>
        <header className={styles.saveHeader}><h2 id="map-edit-dialog-title">另存地图</h2></header>
        <label className={styles.saveLabel} htmlFor="map-edit-name">地图名称</label>
        <div className={styles.saveRow}>
          <input id="map-edit-name" ref={nameRef} aria-describedby={error ? 'map-edit-name-error' : undefined}
            aria-invalid={error ? true : undefined} value={name} maxLength={100}
            onChange={event => { setName(event.target.value); setError('') }} autoComplete="off" spellCheck={false} />
        </div>
        {error && <p id="map-edit-name-error" role="alert" className={styles.error}>{error}</p>}
        <footer className={styles.saveActions}>
          <button type="button" aria-label="取消另存" onClick={() => setDialog(null)}>取消</button>
          <button type="submit" className={`${styles.primary} ${styles.saveSubmit}`} aria-label="另存并重建" disabled={busy}>保存并重建</button>
        </footer>
      </form> : <div><h2 id="map-edit-dialog-title">{phase === 'unconfirmed' ? '离开待确认的保存？' : '放弃未保存修改？'}</h2><p>{phase === 'unconfirmed'
        ? `原地图没有改变。${name} 的保存结果尚未确认，返回编辑时仍可确认。` : '原地图没有改变。本次草稿将被丢弃。'}</p>
        <footer><button type="button" autoFocus onClick={() => { pendingLeave.current = null; setDialog(null) }}>{phase === 'unconfirmed' ? '返回确认' : '继续编辑'}</button><button type="button" className={styles.delete} onClick={() => {
          if (phase !== 'unconfirmed') removeMapDraft(source)
          const proceed = pendingLeave.current; pendingLeave.current = null; setDialog(null); if (proceed) proceed(); else onClose()
        }}>{phase === 'unconfirmed' ? '离开编辑' : '放弃修改'}</button></footer>
      </div>}
    </dialog>
  </div>
})
