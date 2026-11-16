import { useEffect, useRef, useState } from 'react'
import { Check, RotateCw } from 'lucide-react'
import { cancelFollowTask, confirmFollowTarget, getFollowSelection, getFollowTask, type FollowSelection } from '../../services/api'
import type { FollowTaskRef } from '../../types'
import styles from './FollowSelectionCard.module.css'

const reasons: Record<string, string> = {
  waiting_selection_image: '等待新的相机画面', target_not_visible: '人员已离开画面，请刷新重选',
  selection_stale: '画面已失效，请刷新重选', selection_target_invalid: '请选择画面中的人员',
  person_not_verified: '该目标尚未通过真人核验', follow_task_not_active: '这次跟随已结束',
  product_session_mismatch: '机器人会话已变化', follow_task_not_found: '找不到这次跟随任务',
}

export function FollowSelectionCard({ task }: { task: FollowTaskRef }) {
  const [view, setView] = useState<FollowSelection | null>(null)
  const [selected, setSelected] = useState('')
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState('')
  const [revision, setRevision] = useState(0)
  const latest = useRef<FollowSelection | null>(null)
  const { follow_task_id, product_session_id } = task
  useEffect(() => {
    let live = true
    let timer: ReturnType<typeof setTimeout>
    latest.current = null
    const poll = async () => {
      try {
        const ref = { follow_task_id, product_session_id }
        const previous = latest.current
        const next: FollowSelection = previous?.review_state === 'ready'
          ? { ...previous, ...await getFollowTask(ref) }
          : await getFollowSelection(ref, previous === null && revision > 0)
        if (!live) return
        latest.current = next
        setView(next)
        setSelected(value => value || (next.candidates?.some(candidate => candidate.target_id === next.suggested_target_id
          && candidate.kind === 'person') ? next.suggested_target_id! : ''))
        if (!next.found || next.terminal || next.confirmation_required === false) return
      } catch (failure) {
        if (!live) return
        setError(failure instanceof Error ? failure.message : String(failure))
      }
      timer = setTimeout(poll, 1500)
    }
    void poll()
    return () => { live = false; clearTimeout(timer) }
  }, [follow_task_id, product_session_id, revision])

  const pending = view?.found && !view.terminal && view.confirmation_required === true
  const reviewed = view?.review_state === 'ready'
  const candidates = reviewed ? (view.candidates ?? []).filter(item => item.kind !== 'non_person') : []
  const chosen = view?.confirmed_target_id || selected
  const act = async (cancel: boolean) => {
    if (busy || !pending) return
    setBusy(true); setError('')
    try {
      const receipt = cancel ? await cancelFollowTask(task) : await confirmFollowTarget(task, selected, view!.observation_sequence!)
      if (receipt.accepted !== true) throw new Error(reasons[receipt.reason ?? ''] ?? receipt.reason ?? '请求未接受')
      setView(previous => previous ? { ...previous, confirmation_required: false,
        terminal: cancel, confirmed_target_id: cancel ? undefined : selected } : previous)
    } catch (failure) { setError(failure instanceof Error ? failure.message : String(failure)) }
    finally { setBusy(false) }
  }
  const title = view?.terminal ? '选人已结束' : view?.confirmation_required === false ? '已确认跟踪人员'
    : view?.review_state === 'reviewing' ? '正在核验候选' : '确认跟踪对象'
  return <section className={styles.card} aria-label="选人确认卡片">
    <header><strong>{title}</strong><span>确认画面</span></header>
    {view?.image && view.width && view.height ? <div className={styles.frame} style={{ aspectRatio: `${view.width} / ${view.height}` }}>
      <img src={view.image} alt="机器人相机中的候选人员" width={view.width} height={view.height} />
      {candidates.map((candidate, index) => <button key={candidate.target_id} type="button"
        className={styles.box} data-selected={chosen === candidate.target_id} data-uncertain={candidate.kind !== 'person'}
        disabled={!pending || busy || candidate.kind !== 'person'} aria-pressed={chosen === candidate.target_id}
        aria-label={`选择人员 ${index + 1}`} onClick={() => { setSelected(candidate.target_id); setError('') }}
        style={{ left: `${candidate.bbox[0] / view.width! * 100}%`, top: `${candidate.bbox[1] / view.height! * 100}%`,
          width: `${(candidate.bbox[2] - candidate.bbox[0]) / view.width! * 100}%`,
          height: `${(candidate.bbox[3] - candidate.bbox[1]) / view.height! * 100}%` }}>
        <span>{chosen === candidate.target_id && <Check size={12} />}{candidate.kind === 'person' ? `人员 ${index + 1}` : '待核实'}</span>
      </button>)}
    </div> : <div className={styles.waiting}>{reasons[view?.reason ?? ''] ?? (view?.found === false ? '无法读取这次跟随' : '等待候选画面')}</div>}
    {pending && <footer>
      <button type="button" className={styles.confirm} disabled={busy || !selected || !reviewed}
        onClick={() => { void act(false) }}>{busy ? '正在提交…' : '确认跟踪'}</button>
      <button type="button" disabled={busy} onClick={() => { void act(true) }}>取消</button>
      <button type="button" className={styles.refresh} disabled={busy} onClick={() => {
        setSelected(''); setView(null); setError(''); setRevision(value => value + 1)
      }}><RotateCw size={13} />刷新画面</button>
    </footer>}
    {(error || view?.review_state === 'unavailable' || reviewed && !candidates.some(item => item.kind === 'person')) &&
      <p role="status">{error || (view?.review_state === 'unavailable' ? '候选核验不可用，请稍后刷新' : '暂未确认可跟随的人员')}</p>}
  </section>
}
