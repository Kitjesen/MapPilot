import { useCallback, useEffect, useRef, useState } from 'react'
import * as api from '../services/api'
import { isCurrentAgentResponse, presentAgentTask, reconcileCancelRequested } from '../services/agentTask'
import {
  instructionTaskCanDismiss,
  instructionTaskNeedsPolling,
  instructionRejectionMessage,
  type InstructionReceipt,
  type InstructionTask,
} from '../services/instructionTask'

export interface InstructionTaskController {
  task: InstructionTask | null
  receipt: InstructionReceipt | null
  sending: boolean
  error: string
  submit: (text: string) => Promise<void>
  adopt: (runId: string, instruction: string) => void
  cancel: () => Promise<void>
  dismiss: () => void
}

interface InstructionTaskOptions {
  enabled?: boolean
  motionStartAllowed: boolean
  motionStartBlockedReason: string
}

export function useInstructionTask({ motionStartAllowed, motionStartBlockedReason, enabled = true }: InstructionTaskOptions): InstructionTaskController {
  const [task, setTask] = useState<InstructionTask | null>(null)
  const [receipt, setReceipt] = useState<InstructionReceipt | null>(null)
  const [sending, setSending] = useState(false)
  const [error, setError] = useState('')
  const taskRef = useRef<InstructionTask | null>(null)
  const generationRef = useRef(0)
  const sendingRef = useRef(false)
  const pollingRef = useRef(false)
  const mountedRef = useRef(true)

  const updateTask = useCallback((next: InstructionTask | null) => {
    taskRef.current = next
    setTask(next)
  }, [])

  useEffect(() => {
    mountedRef.current = true
    return () => { mountedRef.current = false }
  }, [])

  const runId = task?.runId ?? ''
  const generation = task?.generation ?? 0
  const needsPolling = task ? instructionTaskNeedsPolling(task) : false

  useEffect(() => {
    if (!enabled || !runId || !needsPolling) return
    let disposed = false
    let timer: ReturnType<typeof setTimeout> | undefined
    const poll = async () => {
      if (pollingRef.current) {
        timer = setTimeout(poll, 1_000)
        return
      }
      pollingRef.current = true
      try {
        const response = await api.getAgentTask(runId)
        const current = taskRef.current
        if (disposed || !current || !isCurrentAgentResponse(current, response, generation)) return
        updateTask({
          ...current, status: response, statusSource: 'poll', pollError: '',
          cancelRequested: response.state === 'done' ? false
            : reconcileCancelRequested(current.cancelRequested, response),
        })
      } catch (failure: unknown) {
        const current = taskRef.current
        if (!disposed && current?.generation === generation) {
          updateTask({ ...current, pollError: failure instanceof Error ? failure.message : String(failure) })
        }
      } finally {
        pollingRef.current = false
        const current = taskRef.current
        if (!disposed && current?.generation === generation && instructionTaskNeedsPolling(current)) {
          timer = setTimeout(poll, 1_000)
        }
      }
    }
    timer = setTimeout(poll, 1_000)
    return () => { disposed = true; if (timer) clearTimeout(timer) }
  }, [runId, generation, needsPolling, updateTask, enabled])

  const submit = useCallback(async (input: string) => {
    const text = input.trim()
    const current = taskRef.current
    const rejection = !text ? '请输入任务指令。'
      : sendingRef.current ? '正在发送指令，请稍候。' : ''
    if (rejection) { setError(rejection); throw new Error(rejection) }
    const submissionGeneration = generationRef.current
    sendingRef.current = true
    setSending(true)
    setError('')
    try {
      const controlOnly = !motionStartAllowed || Boolean(current && !instructionTaskCanDismiss(current))
      const response = await api.sendInstruction(text, current?.runId, controlOnly)
      if (!mountedRef.current || generationRef.current !== submissionGeneration) return
      if (!response.ok || response.accepted === false) {
        throw new Error(instructionRejectionMessage(response.reason)
          ?? response.reason ?? (!motionStartAllowed && response.operation === 'start'
            ? motionStartBlockedReason : api.formatCommandAck(response, '指令')))
      }
      const operation = response.operation
      if (operation !== 'start') {
        if (operation !== 'query' && operation !== 'keep' && operation !== 'cancel') {
          throw new Error(response.reason ?? '当前任务不支持这项控制。')
        }
        const latest = taskRef.current
        if (current) {
          if (!latest || latest.runId !== current.runId || latest.generation !== current.generation) return
          if (response.run_id !== current.runId || (response.task_status && response.task_status.run_id !== current.runId)) {
            throw new Error('指令响应与当前任务不匹配。')
          }
          // A control acknowledgement cannot downgrade a newer terminal poll.
          if (!instructionTaskCanDismiss(current) && instructionTaskCanDismiss(latest)) return
          const status = response.task_status ?? latest.status
          const cancelRequested = operation === 'cancel' ? true : latest.cancelRequested
          updateTask({ ...latest, status,
            statusSource: response.task_status ? 'poll' : latest.statusSource,
            cancelRequested: status.state === 'done' ? false : reconcileCancelRequested(cancelRequested, status),
            cancelError: '', pollError: response.task_status ? '' : latest.pollError })
        } else {
          const snapshot = response.task_status
          const idle = !response.run_id && response.state === 'idle'
          if (response.run_id && snapshot?.run_id === response.run_id && snapshot.found) {
            updateTask({ runId: response.run_id, instruction: snapshot.instruction || response.instruction || '当前任务',
              executor: response.executor ?? 'agent', generation: submissionGeneration,
              status: snapshot, statusSource: 'poll',
              cancelRequested: snapshot.state === 'done' ? false : reconcileCancelRequested(operation === 'cancel', snapshot),
              cancelError: '', pollError: '' })
            setReceipt(null)
            return
          }
          setReceipt({ instruction: text, executor: response.executor ?? '',
            operation, message: idle ? '当前没有正在执行的任务。'
              : snapshot ? snapshot.summary || presentAgentTask(snapshot).detail
                : '任务服务已响应；当前执行状态尚未确认。' })
        }
        return
      }
      if (response.accepted !== true) throw new Error('任务服务未明确接受新任务。')
      const nextGeneration = ++generationRef.current
      setReceipt(null)
      if (response.run_id) {
        updateTask({
          runId: response.run_id, instruction: text, executor: response.executor ?? 'agent',
          generation: nextGeneration,
          status: { found: true, run_id: response.run_id, state: response.state ?? 'accepted' },
          statusSource: 'ack', cancelRequested: false, cancelError: '', pollError: '',
        })
      } else {
        updateTask(null)
        setReceipt({ instruction: text, executor: response.executor ?? '', operation: 'start',
          message: '指令已受理，执行器未返回可查询的任务 ID；执行结果尚未确认。' })
      }
    } catch (failure: unknown) {
      const rejectionMessage = api.isGatewayApiError(failure)
        ? instructionRejectionMessage(failure.body?.reason) : undefined
      const message = api.formatCommandError(rejectionMessage ? new Error(rejectionMessage) : failure, '指令发送失败')
      if (mountedRef.current) setError(message)
      throw new Error(message)
    } finally {
      sendingRef.current = false
      if (mountedRef.current) setSending(false)
    }
  }, [motionStartAllowed, motionStartBlockedReason, updateTask])

  const cancel = useCallback(async () => {
    const current = taskRef.current
    if (!current || current.cancelRequested || instructionTaskCanDismiss(current)) return
    updateTask({ ...current, cancelRequested: true, cancelError: '' })
    try {
      const response = await api.cancelAgentTask(current.runId)
      const latest = taskRef.current
      if (!mountedRef.current || !latest || latest.runId !== current.runId || latest.generation !== current.generation) return
      if (instructionTaskCanDismiss(latest)) return
      if (response.run_id && response.run_id !== current.runId) {
        updateTask({ ...latest, cancelRequested: false, cancelError: '任务结束响应与当前任务不匹配。' })
      } else if (!response.ok || response.cancel_requested !== true) {
        updateTask({ ...latest, cancelRequested: false, cancelError: response.reason ?? '当前任务未接受结束请求。' })
      } else {
        const status = { ...latest.status,
          stop_confirmed: latest.status.stop_confirmed === true || response.stop_confirmed === true }
        updateTask({ ...latest, cancelRequested: reconcileCancelRequested(true, status), status })
      }
    } catch (failure: unknown) {
      const latest = taskRef.current
      if (mountedRef.current && latest?.generation === current.generation) {
        updateTask({ ...latest, cancelRequested: false,
          cancelError: failure instanceof Error ? failure.message : String(failure) })
      }
    }
  }, [updateTask])

  const adopt = useCallback((runId: string, instruction: string) => {
    if (!runId || taskRef.current?.runId === runId) return
    const generation = ++generationRef.current
    setReceipt(null)
    setError('')
    updateTask({ runId, instruction, executor: 'agent', generation,
      status: { found: true, run_id: runId, state: 'accepted' }, statusSource: 'ack',
      cancelRequested: false, cancelError: '', pollError: '' })
  }, [updateTask])

  const dismiss = useCallback(() => {
    const current = taskRef.current
    if (sendingRef.current || (current && !instructionTaskCanDismiss(current))) return
    ++generationRef.current
    updateTask(null)
    setReceipt(null)
    setError('')
  }, [updateTask])

  return { task, receipt, sending, error, submit, adopt, cancel, dismiss }
}
