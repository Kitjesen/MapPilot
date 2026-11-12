import { forwardRef, useCallback, useEffect, useImperativeHandle, useMemo, useRef, useState, type PointerEvent as ReactPointerEvent } from 'react'
import { createPortal } from 'react-dom'
import { ArrowRight, BoxSelect, BrushCleaning, ChevronDown, GripVertical, MoreHorizontal, Redo2, SlidersHorizontal, Trash2, Undo2, X } from 'lucide-react'
import type { Bounds } from '../types/stairs.ts'
import type { MapEditOperation, MapCropKind, MapFilterOperation, MapPlaneOperation, MapEditPreview, MapEditScene, MapEditSource, MapEditView } from '../types/mapEdit.ts'
import { MapEditSaveUnconfirmed, applyMapEditOperations, isLocalMapEdit, mapEditLabel, validMapEditOperation, mapEditCloudBounds, mapEditNameError, mapEditSelectionImpact, suggestedEditedMapName, validMapEditBounds } from '../services/mapEdit.ts'
import { readMapDraft, removeMapDraft, writeMapDraft } from '../services/mapDraft.ts'
import type { MapDraftResult, MapEditDraft } from '../services/mapDraft.ts'
import * as api from '../services/api.ts'
import { useMapSettings } from '../hooks/useMapSettings.ts'
import { MapProgress } from './MapProgress.tsx'
import { putNoGo } from '../services/noGo.ts'
import styles from './MapEditor.module.css'

export interface MapEditorHandle { setBounds(bounds: Bounds): void }
export type MapEditorTool = 'crop' | 'clean' | 'surface' | 'region' | 'point'
interface Props {
  source: MapEditSource
  points: ArrayLike<number>
  anchor: { x: number; y: number } | null
  footerTarget: HTMLElement | null
  cameraExpanded?: boolean
  initialTool?: MapEditorTool
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

const filters: MapFilterOperation[] = [
  { operation: 'voxel', voxel_size: .1 },
  { operation: 'statistical', neighbors: 20, std_ratio: 1.5 },
  { operation: 'radius', radius: .2, min_points: 12 },
  { operation: 'clusters', radius: .2, neighbors: 20, min_cluster_points: 100 },
  { operation: 'plane_crop', distance_threshold: .05, margin: .5 },
]
const filterFields: Record<string, { label: string; step: number; min: number }> = {
  voxel_size: { label: '体素大小 m', step: .01, min: .01 },
  neighbors: { label: '邻点数', step: 1, min: 1 },
  std_ratio: { label: '标准差倍数', step: .1, min: .1 },
  radius: { label: '邻域半径 m', step: .01, min: .01 },
  min_points: { label: '最少邻点', step: 1, min: 1 },
  min_cluster_points: { label: '最小簇点数', step: 1, min: 1 },
  distance_threshold: { label: '平面距离阈值 m', step: .001, min: .001 },
  margin: { label: '范围扩展 m', step: .05, min: 0 },
}

export const MapEditor = forwardRef<MapEditorHandle, Props>(function MapEditor(
  { source, points, anchor, footerTarget, cameraExpanded = false, initialTool, onExitGuardChange, onSceneChange, onView, onClose, onOpenMap, onMapsChanged }, ref,
) {
  const { name: sourceName, contentEpoch, frameId } = source
  const [storedDraft] = useState(() => readMapDraft(source))
  const [recovery, setRecovery] = useState<MapDraftResult | null>(() => storedDraft?.kind === 'unconfirmed' ? null : storedDraft)
  const [pendingSave, setPendingSave] = useState<MapEditDraft | null>(() => storedDraft?.kind === 'unconfirmed' ? storedDraft.draft : null)
  const [history, setHistory] = useState<{ edits: MapEditOperation[]; cursor: number }>({ edits: [], cursor: 0 })
  const operations = useMemo(() => history.edits.slice(0, history.cursor), [history])
  const [selection, setSelection] = useState<Bounds | null>(null)
  const [selecting, setSelecting] = useState(initialTool === 'crop' || initialTool === 'surface' || initialTool === 'region')
  const [regionName, setRegionName] = useState('')
  const [regionId] = useState(() => api.makeRequestId('region'))
  const [regionKind, setRegionKind] = useState<'no_go' | 'glass'>('no_go')
  const [planeAxis, setPlaneAxis] = useState<'x' | 'y' | 'z'>('z')
  const [planeOffset, setPlaneOffset] = useState<number | null>(null)
  const [planeSpacing, setPlaneSpacing] = useState(.05)
  const surface = useMemo<MapPlaneOperation | null>(() => initialTool === 'surface' && selection ? {
    operation: 'add_plane', bounds: selection, axis: planeAxis,
    offset: planeOffset ?? selection.min[planeAxis], spacing: planeSpacing,
  } : null, [initialTool, selection, planeAxis, planeOffset, planeSpacing])
  const surfaceValid = surface && validMapEditOperation(surface)
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
  const [selectionSize, setSelectionSize] = useState({ width: 280, height: 300 })
  const [selectionPosition, setSelectionPosition] = useState<{ left: number; top: number } | null>(null)
  const [draggingSelection, setDraggingSelection] = useState(false)
  const [recordOpen, setRecordOpen] = useState(false)
  const mapSettings = useMapSettings()
  const [filter, setFilter] = useState<MapFilterOperation>(filters[0])
  const [previewState, setPreviewState] = useState<{
    operations: readonly MapEditOperation[]; result?: MapEditPreview; error?: string
  } | null>(null)
  const [previewRetry, setPreviewRetry] = useState(0)
  const rootRef = useRef<HTMLDivElement>(null)
  const footerRef = useRef<HTMLDivElement>(null)
  const selectionRef = useRef<HTMLElement>(null)
  const selectionDrag = useRef<{ startX: number; startY: number; left: number; top: number } | null>(null)
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
  const nativePreview = operations.some(edit => !isLocalMapEdit(edit))
  const currentPreview = nativePreview && previewState?.operations === operations ? previewState : null
  const preview = currentPreview?.result ?? null
  const previewPending = nativePreview && !currentPreview && !recovery && !pendingSave
  const footerVisible = Boolean(history.edits.length || recovery || pendingSave || busy || saved || error || nativePreview)
  const previewError = currentPreview?.error
  const saveReady = !nativePreview || !!preview && preview.point_count > 0
  const draftPoints = useMemo(() => nativePreview ? preview?.preview_points ?? []
    : operations.length ? applyMapEditOperations(points, operations) : points, [points, operations, nativePreview, preview])
  const selectionImpact = useMemo(() => selection ? mapEditSelectionImpact(draftPoints, selection) : null, [draftPoints, selection])
  const fullHeight = selection && cloudBounds && selection.min.z <= cloudBounds.min.z && selection.max.z >= cloudBounds.max.z
  const savedCounts = saved ? [
    typeof saved.pointCount === 'number' ? `保留 ${pointCount.format(saved.pointCount)} 点` : null,
    typeof saved.removedPoints === 'number' ? `删除 ${pointCount.format(saved.removedPoints)} 点` : null,
  ].filter(Boolean).join(' · ') : ''

  useEffect(() => { alive.current = true; return () => { alive.current = false } }, [])
  const setSharedSelection = mapSettings?.setSelection
  useEffect(() => {
    setSharedSelection?.(selection)
    return () => setSharedSelection?.(null)
  }, [selection, setSharedSelection])
  const setSettingsEditing = mapSettings?.setEditing
  useEffect(() => {
    setSettingsEditing?.(true)
    return () => setSettingsEditing?.(false)
  }, [setSettingsEditing])
  useEffect(() => {
    if (!nativePreview || recovery || pendingSave || saved) return
    const controller = new AbortController()
    let active = true
    void api.previewEditedMap({ name: sourceName, contentEpoch, frameId }, operations, controller.signal).then(
      result => { if (active) setPreviewState({ operations, result }) },
      reason => { if (active) setPreviewState({ operations, error: reason instanceof Error ? reason.message : '清洗预览失败' }) },
    )
    return () => { active = false; controller.abort() }
  }, [sourceName, contentEpoch, frameId, operations, nativePreview, recovery, pendingSave, saved, previewRetry])
  const openSettings = mapSettings?.open
  useEffect(() => {
    selectRef.current?.focus()
    if (initialTool === 'clean') openSettings?.('cloud')
  }, [initialTool, openSettings])
  useEffect(() => {
    if (recovery || pendingSave || saved || history.edits.length === 0) return
    writeMapDraft({ source: { name: sourceName, contentEpoch, frameId }, name, history, status: 'editing' })
  }, [sourceName, contentEpoch, frameId, name, history, recovery, pendingSave, saved])
  useEffect(() => {
    const element = rootRef.current
    if (!element) return
    const observer = new ResizeObserver(() => {
      const canvas = element.getBoundingClientRect()
      const footer = (footerRef.current ?? footerTarget?.parentElement)?.getBoundingClientRect()
      const bottom = canvas.height - (canvas.width < 760 ? 74 : 12)
      setViewport({ width: canvas.width, height: canvas.height, bottom: footer?.height
        ? Math.min(bottom, footer.top - canvas.top - 12) : bottom })
    })
    observer.observe(element)
    if (footerRef.current) observer.observe(footerRef.current)
    if (footerTarget) observer.observe(footerTarget)
    if (footerTarget?.parentElement) observer.observe(footerTarget.parentElement)
    return () => observer.disconnect()
  }, [footerTarget, footerVisible])
  useEffect(() => {
    onSceneChange({ selecting: !locked && selecting, selection, operations, view, showDeleted, preview,
      surface: !locked && view === 'draft' && surfaceValid ? surface : null })
  }, [selecting, selection, operations, view, showDeleted, locked, preview, surface, surfaceValid, onSceneChange])
  useEffect(() => {
    const element = selectionRef.current
    if (!selection || !element) return
    const observer = new ResizeObserver(() => setSelectionSize({ width: element.offsetWidth, height: element.scrollHeight + 2 }))
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
    setSelection(copyBounds(bounds)); setPlaneOffset(null); setSelecting(false); setError(''); setView('draft')
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
        && event.target.closest('input, textarea, select, [contenteditable="true"], [aria-modal="true"]'))) return
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
  const apply = (operation: MapCropKind) => {
    if (!selection || !validMapEditBounds(selection) || locked || previewPending) return
    const edit = { operation, bounds: copyBounds(selection) }
    const next = [...operations, edit]
    if (!nativePreview && applyMapEditOperations(draftPoints, [edit]).length === 0) {
      setError('该操作会清空预览中的点云，请缩小范围。'); return
    }
    setHistory({ edits: next, cursor: next.length }); setSelection(null); setSelecting(false); setError(''); setView('draft')
    selectRef.current?.focus()
  }
  const applyFilter = () => {
    if (locked || previewPending) return
    if (!validMapEditOperation(filter)) { setError('请填写有效的清洗参数。'); return }
    const next = [...operations, { ...filter }]
    setHistory({ edits: next, cursor: next.length })
    setSelection(null); setSelecting(false); setError(''); setView('draft')
  }
  const applySurface = () => {
    if (locked || previewPending || !surfaceValid || !surface) return
    const next = [...operations, { ...surface, bounds: copyBounds(surface.bounds) }]
    setHistory({ edits: next, cursor: next.length })
    setSelection(null); setSelecting(false); setError(''); setView('draft')
  }
  const saveRegion = async () => {
    if (!selection || locked || !regionName.trim() || mapSettings?.map?.active) return
    setPhase('saving'); setError('')
    try {
      await putNoGo(sourceName, contentEpoch, { id: regionId, label: regionName.trim(), kind: regionKind,
        min: [selection.min.x, selection.min.y, selection.min.z], max: [selection.max.x, selection.max.y, selection.max.z] })
      onMapsChanged(); onClose()
    } catch (reason) { setError(reason instanceof Error ? reason.message : '区域保存失败') }
    finally { if (alive.current) setPhase('editing') }
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
    if (!pendingSave && !saveReady) return
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
  const clampSelectionPosition = (left: number, top: number) => ({
    left: Math.max(12, Math.min(left, viewport.width - Math.min(selectionSize.width, viewport.width - 24) - 12)),
    top: Math.max(12, Math.min(top, viewport.bottom - Math.min(selectionSize.height, viewport.bottom - 12))),
  })
  const position = clampSelectionPosition(
    selectionPosition?.left ?? (anchor?.x ?? viewport.width / 2) + 18,
    selectionPosition?.top ?? (anchor?.y ?? 100) + 18,
  )
  const beginSelectionDrag = (event: ReactPointerEvent<HTMLElement>) => {
    if (event.button !== 0 || (event.target as Element).closest('button:not([data-selection-drag])')) return
    event.preventDefault()
    event.stopPropagation()
    event.currentTarget.setPointerCapture(event.pointerId)
    selectionDrag.current = { startX: event.clientX, startY: event.clientY, ...position }
    setDraggingSelection(true)
  }
  const moveSelectionDrag = (event: ReactPointerEvent<HTMLElement>) => {
    const drag = selectionDrag.current
    if (!drag) return
    setSelectionPosition(clampSelectionPosition(drag.left + event.clientX - drag.startX, drag.top + event.clientY - drag.startY))
  }
  const endSelectionDrag = () => {
    selectionDrag.current = null
    setDraggingSelection(false)
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
      <button type="button" onClick={() => { if (isLocalMapEdit(op)) { setSelection(copyBounds(op.bounds)); setSelecting(false); onView('three') } else { setFilter(op); mapSettings?.open() } }}>
        {index + 1}. {mapEditLabel(op)}</button>
    </li>) : <li>尚无修改</li>}</ol>}
    <p>{source.name}<br />地图版本 {source.contentEpoch} · {source.frameId}<br />已加载完整点云，修改另存为新地图。</p>
  </>
  const footer = footerVisible && <div ref={footerRef} className={styles.footerStack}>
    {error && dialog !== 'save' && <p role="alert" className={styles.error}>{error}</p>}
    {previewError && <div role="alert" className={styles.error}>{previewError}<button type="button" onClick={() => { setPreviewState(null); setPreviewRetry(value => value + 1) }}>重试预览</button></div>}
    <div className={styles.bar} aria-label="地图编辑草稿" aria-busy={busy || previewPending}>
      <div className={styles.barSummary}>
        <strong role="status">{recovery ? recovery.kind === 'ready' ? '发现未保存草稿' : '原地图已更新' : phase === 'saving' ? '正在保存点云' : phase === 'building' ? '正在重建导航数据'
          : previewPending ? '正在计算清洗预览' : previewError ? '清洗预览失败' : phase === 'unconfirmed' ? '保存结果待确认' : phase === 'build_failed' ? '点云已保存 · 重建失败' : phase === 'saved' ? '新地图已保存' : operations.length ? '未保存修改' : '尚无修改'}</strong>
        {recovery && <span className={styles.savedCounts}>{recovery.kind === 'ready'
          ? `${recovery.draft.history.cursor} 步修改${recovery.draft.history.edits.length > recovery.draft.history.cursor
            ? ` · ${recovery.draft.history.edits.length - recovery.draft.history.cursor} 步可重做` : ''} · ${recovery.draft.name}`
          : '旧草稿不能应用到当前地图'}</span>}
        {phase !== 'editing' && <span className={styles.targetName} title={saved?.name ?? name.trim()}>{saved?.name ?? name.trim()}</span>}
        {phase === 'editing' && preview && <span className={styles.savedCounts}>完整点云保留 {pointCount.format(preview.point_count)} 点 · 删除 {pointCount.format(preview.removed_points)} 点{preview.preview_sample ? ' · 画面为采样预览' : ''}</span>}
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
            <button type="button" ref={saveRef} className={styles.primary} aria-label="另存新地图" disabled={busy || operations.length === 0 || !!saved || !saveReady}
              onClick={() => { setError(''); setDialog('save') }}><span className={styles.saveLong}>另存新地图</span><span className={styles.saveShort}>另存</span><ArrowRight size={18} /></button>
          </>}
    </div>
  </div>

  return <div className={styles.editor} ref={rootRef} data-map-editor>
    <div className={styles.toolbar} role="group" aria-label="点云整理工具">
      <strong className={styles.mode}>点云整理</strong>
      <button type="button" ref={selectRef} aria-label="框选范围" aria-pressed={selecting} title="框选三维范围" disabled={locked} onClick={startSelection}><BoxSelect size={20} /><span className={styles.toolLabel}>框选</span></button>
      <button type="button" className={styles.numeric} aria-label="输入选区范围" title="输入选区范围" disabled={locked} onClick={numericSelection}><SlidersHorizontal size={18} /></button>
      {initialTool !== 'region' && <button type="button" aria-label="清洗点云" title="配置点云清洗并预览" onClick={() => mapSettings?.open('cloud')}><BrushCleaning size={18} /><span className={styles.toolLabel}>清洗</span></button>}
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
    {initialTool !== 'region' && mapSettings?.target && createPortal(<section className={styles.filters} aria-label="点云清洗参数">
      <div className={styles.filterChoices} role="group" aria-label="清洗方式">{filters.map(item => <button key={item.operation} type="button"
        aria-pressed={filter.operation === item.operation} onClick={() => setFilter({ ...item })}>{mapEditLabel(item)}</button>)}</div>
      <form onSubmit={event => { event.preventDefault(); applyFilter() }}>
        <div className={styles.filterFields}>{Object.entries(filter).filter(([key]) => key !== 'operation').map(([key, value]) => {
          const field = filterFields[key]
          return <label key={`${filter.operation}-${key}`}><span>{field.label}</span><input type="number" value={Number.isFinite(value) ? value : ''}
            min={field.min} step={field.step} required onChange={event => setFilter({ ...filter, [key]: event.currentTarget.valueAsNumber })} /></label>
        })}</div>
        <p>{filter.operation === 'plane_crop' ? '平面范围裁剪（不补地面）。保留拟合平面附近包围盒中的原有点。' : '查看预览后再另存。稀疏玻璃、细杆和地面也可能被筛除。'}</p>
        <button className={styles.filterApply} type="submit" disabled={locked || previewPending}>应用并预览</button>
      </form>
      {(busy || previewPending) && <MapProgress key={phase === 'editing' ? 'preview' : phase}
        stage={phase === 'saving' ? '保存点云 · 第 1 / 2 步' : phase === 'building' ? '重建导航地图 · 第 2 / 2 步' : '计算完整点云清洗预览'}
        completed={phase === 'building' ? '点云已保存' : undefined} />}
      {preview && <p role="status">完整点云保留 {pointCount.format(preview.point_count)} 点，删除 {pointCount.format(preview.removed_points)} 点。</p>}
      {previewError && <p role="alert">{previewError}</p>}
    </section>, mapSettings.target)}
    {(busy || previewPending) && <div className={styles.processing}><MapProgress key={phase === 'editing' ? 'preview' : phase}
      stage={initialTool === 'region' ? '保存区域标注' : phase === 'saving' ? '保存点云 · 第 1 / 2 步' : phase === 'building' ? '重建导航地图 · 第 2 / 2 步' : '计算完整点云清洗预览'}
      completed={phase === 'building' ? '点云已保存' : undefined} /></div>}
    {selecting && <p className={styles.hint} role="status">当前视角拖动框选 · 橙色框为三维选区 · Esc 取消</p>}
    {selection && <section ref={selectionRef} className={`${styles.selection} ${draggingSelection ? styles.draggingSelection : ''}`}
      style={{ ...position, maxHeight: Math.max(0, viewport.bottom - position.top) }} aria-label="调整选区范围"
      onPointerDown={event => event.stopPropagation()}>
      <header onPointerDown={beginSelectionDrag} onPointerMove={moveSelectionDrag} onPointerUp={endSelectionDrag}
        onPointerCancel={endSelectionDrag} onLostPointerCapture={endSelectionDrag}>
        <button type="button" className={styles.selectionHandle} data-selection-drag aria-label="移动选区面板"
          title="拖动面板，也可用方向键移动" onKeyDown={event => {
            const directions: Record<string, readonly [number, number]> = { ArrowLeft: [-16, 0], ArrowRight: [16, 0], ArrowUp: [0, -16], ArrowDown: [0, 16] }
            const direction = directions[event.key]
            if (!direction) return
            event.preventDefault()
            event.stopPropagation()
            setSelectionPosition(clampSelectionPosition(position.left + direction[0], position.top + direction[1]))
          }}><GripVertical size={15} aria-hidden="true" /><strong>选区高度</strong><span>地图 Z</span></button>
        <button type="button" aria-label="取消本次选区" onClick={() => setSelection(null)}><X size={17} /></button>
      </header>
      {selectionImpact && <div className={styles.selectionImpact} aria-label="选区预览统计">
        <p><span>{preview?.preview_sample ? '预览样本' : '当前点云'}</span><strong>选中 {pointCount.format(selectionImpact.selected)} 点 · {pointPercent.format(selectionImpact.total ? selectionImpact.selected / selectionImpact.total : 0)}</strong></p>
        {fullHeight && <p className={styles.heightHint}>覆盖整段高度 · 请留意同位置的地面</p>}
      </div>}
      {initialTool === 'region' && <div className={styles.filters}>
        <label>区域名称<input aria-label="区域名称" maxLength={128} placeholder="如东侧玻璃门" value={regionName}
          onChange={event => setRegionName(event.target.value)} /></label>
        <label>区域类型<select aria-label="区域类型" value={regionKind} onChange={event => setRegionKind(event.target.value as 'no_go' | 'glass')}>
          <option value="no_go">禁行区域</option><option value="glass">玻璃障碍</option></select></label>
        <p>两类标注都禁止机器人进入此三维范围。保存后重新加载导航地图生效；不会删除实测点云。</p>
      </div>}
      {initialTool === 'surface' && <div className={styles.filters}>
        <label>补面方向<select aria-label="补面方向" value={planeAxis} onChange={event => {
          setPlaneAxis(event.target.value as 'x' | 'y' | 'z'); setPlaneOffset(null)
        }}><option value="z">地面（XY）</option><option value="x">墙面（YZ）</option><option value="y">墙面（XZ）</option></select></label>
        <div className={styles.filterFields}>
          <label><span>{planeAxis.toUpperCase()} 坐标 m</span><input type="number" aria-label="补面坐标" step="0.01"
            value={surface?.offset ?? ''} onChange={event => setPlaneOffset(event.currentTarget.valueAsNumber)} /></label>
          <label><span>点间距 m</span><input type="number" aria-label="补面点间距" min="0.01" step="0.01" value={planeSpacing}
            onChange={event => setPlaneSpacing(event.currentTarget.valueAsNumber)} /></label>
        </div>
        <p>橙色为人工补面预览。记录人工来源，原图保留；不会推断周围空间已被扫描。</p>
        {!surfaceValid && <p role="alert">补面坐标须在选区内，间距须大于零，单次最多 100 万点。</p>}
      </div>}
      <div className={styles.coordinates}>{coordinates('z')}</div>
      <div className={styles.views} role="group" aria-label="检查选区视角">{(['top', 'front', 'side', 'three'] as const).map(value =>
        <button key={value} type="button" onClick={() => onView(value)}>{({ top: '俯视', front: '正视', side: '侧视', three: '三维' })[value]}</button>)}</div>
      <details className={styles.bounds}><summary>调整平面范围</summary><div className={styles.coordinates}>{coordinates('x')}{coordinates('y')}</div></details>
      {error && <p role="alert" className={styles.error}>{error}</p>}
      <div className={styles.selectionActions}>{initialTool === 'region'
        ? <button type="button" disabled={locked || !regionName.trim() || mapSettings?.map?.active} onClick={() => void saveRegion()}>保存区域标注</button>
        : initialTool === 'surface'
        ? <button type="button" disabled={locked || previewPending || !surfaceValid} onClick={applySurface}>添加补面到草稿</button>
        : <button type="button" className={styles.delete} disabled={locked || previewPending || (!nativePreview && !selectionImpact?.selected)} onClick={() => apply('delete_inside')}><Trash2 size={16} />删除选中</button>}
        {initialTool !== 'region' && <details className={styles.more}><summary aria-label="更多选区操作"><MoreHorizontal size={20} /></summary><div className={styles.menu}><button type="button" disabled={locked || previewPending} onClick={() => apply('keep_inside')}>裁掉框外，仅保留选区</button></div></details>}
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
          <button type="submit" className={`${styles.primary} ${styles.saveSubmit}`} aria-label="另存并重建" disabled={busy || !saveReady}>保存并重建</button>
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
