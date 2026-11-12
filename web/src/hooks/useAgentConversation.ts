import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { useMapSettings } from './useMapSettings.ts'
import { makeRequestId } from '../services/api'
import { readBotProfile, botPersona } from '../services/botProfile.ts'
import type { AgentMessageEvent } from '../types'
import {
  ConversationError, createConversation, createConversationShare, deleteConversationShare, fetchConversation, fetchConversationOptions,
  isConversationAttachment, isConversationPermission, listConversations, searchConversations, parseConversationMessageEvent, mergeConversationMessages, resolveConversationApproval, editQueuedMessage, promoteQueuedMessage,
  sendConversationMessage, updateConversation, uploadConversationAttachment, forkConversation as createConversationFork,
  type ConversationAttachment, type ConversationMapContext, type ConversationMetadata, type ConversationOptions, type ConversationPermission, type ConversationSearchResult, type ConversationSnapshot, type ConversationSubmission,
} from '../services/agentConversation'

interface StoredConversation {
  conversation_id: string
  draft: string
  replyTo: string
  replyQuote: string
  pending: ConversationSubmission | null
  editable?: boolean
  model: string
  reasoning: string
  permissionMode: ConversationPermission
  attachments: ConversationAttachment[]
  queuedEdit?: { id: string; text: string } | null
  includeMapContext: boolean
  includeSelection: boolean
}

function readStoredConversation(key = `lingtu.agent.conversation:${location.origin}`): StoredConversation {
  try {
    const stored = JSON.parse(localStorage.getItem(key) ?? 'null')
    if (stored && typeof stored.conversation_id === 'string' && typeof stored.draft === 'string') {
      const pending = stored.pending
      return { conversation_id: stored.conversation_id, draft: stored.draft, replyTo: typeof stored.replyTo === 'string' ? stored.replyTo : '', editable: stored.editable === true,
        includeMapContext: stored.includeMapContext !== false, includeSelection: stored.includeSelection !== false,
        queuedEdit: stored.queuedEdit && typeof stored.queuedEdit.id === 'string' && typeof stored.queuedEdit.text === 'string'
          ? stored.queuedEdit : null,
        replyQuote: typeof stored.replyQuote === 'string' ? stored.replyQuote : '',
        model: typeof stored.model === 'string' ? stored.model : '', reasoning: typeof stored.reasoning === 'string' ? stored.reasoning : '',
        permissionMode: isConversationPermission(stored.permissionMode) ? stored.permissionMode : 'default',
        attachments: Array.isArray(stored.attachments) ? stored.attachments.filter(isConversationAttachment) : [],
        pending: pending && typeof pending.message_id === 'string' && typeof pending.text === 'string'
          && ['model', 'reasoning', 'reply_to_message_id', 'reply_quote'].every(key => pending[key] == null || typeof pending[key] === 'string')
          && (pending.persona == null || typeof pending.persona === 'string' && pending.persona.length <= 2000)
          && (pending.attachment_ids == null || (Array.isArray(pending.attachment_ids)
            && pending.attachment_ids.every((id: unknown) => typeof id === 'string')))
          ? { message_id: pending.message_id, text: pending.text, model: pending.model, reasoning: pending.reasoning, enqueue: pending.enqueue === true,
            ...(pending.persona ? { persona: pending.persona } : {}),
            reply_to_message_id: pending.reply_to_message_id, attachment_ids: pending.attachment_ids, map_context: pending.map_context,
            ...(pending.reply_quote ? { reply_quote: pending.reply_quote } : {}),
            permission_mode: isConversationPermission(pending.permission_mode) ? pending.permission_mode : 'default' } : null }
    }
  } catch { /* Storage can be unavailable in a browser privacy session. */ }
  return { conversation_id: '', draft: '', replyTo: '', replyQuote: '', pending: null, model: '', reasoning: '', permissionMode: 'default', attachments: [], includeMapContext: true, includeSelection: true }
}

export function useAgentConversation(onRunObserved?: (runId: string, instruction: string) => void, enabled = true, liveMessage?: AgentMessageEvent | null) {
  const mapSettings = useMapSettings()
  const mapContext = useRef<ConversationSubmission['map_context']>(null)
  const [stored, setStored] = useState(readStoredConversation)
  const map = mapSettings?.map
  const selection = mapSettings?.selection
  const context = useMemo<ConversationMapContext | null>(() => map ? { map_id: map.name, content_epoch: map.contentEpoch,
    selection: selection ? { min: [selection.min.x, selection.min.y, selection.min.z],
      max: [selection.max.x, selection.max.y, selection.max.z] } : null } : null, [map, selection])
  useEffect(() => {
    mapContext.current = context && stored.includeMapContext
      ? { ...context, selection: stored.includeSelection ? context.selection : null } : null
  }, [context, stored.includeMapContext, stored.includeSelection])
  const storedRef = useRef(stored)
  const [snapshot, setSnapshot] = useState<ConversationSnapshot | null>(null)
  const snapshotRef = useRef<ConversationSnapshot | null>(null)
  const [open, setOpen] = useState(false)
  const [sessionListOpen, setSessionListOpen] = useState(false)
  const [conversations, setConversations] = useState<ConversationMetadata[]>([])
  const [searchQuery, setSearchQuery] = useState('')
  const [searchResults, setSearchResults] = useState<ConversationSearchResult[]>([])
  const [searchBusy, setSearchBusy] = useState(false)
  const [searchError, setSearchError] = useState('')
  const [searchTarget, setSearchTarget] = useState<{ conversationId: string; messageId: string; query: string } | null>(null)
  const [archivedView, setArchivedView] = useState(false)
  const [sessionsLoading, setSessionsLoading] = useState(false)
  const [sessionBusy, setSessionBusy] = useState<string | null>(null)
  const sessionBusyRef = useRef<string | null>(null)
  const sessionsRequest = useRef(0)
  const sessionEpoch = useRef(0)
  const mutationEpoch = useRef(0)
  const [historyLoading, setHistoryLoading] = useState(false)
  const historyRequest = useRef<Promise<boolean> | null>(null)
  const expandedHistory = useRef<string | null>(null)
  const [shareBusy, setShareBusy] = useState(false)
  const shareBusyRef = useRef(false)
  const [sending, setSending] = useState(false)
  const [loading, setLoading] = useState(false)
  const [error, setError] = useState('')
  const [refreshError, setRefreshError] = useState('')
  const [notice, setNotice] = useState('')
  const [options, setOptions] = useState<ConversationOptions | null>(null)
  const optionsRef = useRef<ConversationOptions | null>(null)
  const [optionsError, setOptionsError] = useState('')
  const [optionsLoading, setOptionsLoading] = useState(false)
  const optionsRequest = useRef<Promise<void> | null>(null)
  const [uploading, setUploading] = useState(false)
  const [approvalBusy, setApprovalBusy] = useState<string | null>(null)
  const approvalBusyRef = useRef<string | null>(null)
  const uploadingRef = useRef(false)
  const mounted = useRef(true)
  const sendingRef = useRef(false)
  const refreshRef = useRef<Promise<void> | null>(null)
  const observedRuns = useRef(new Set<string>())
  const runObserver = useRef(onRunObserved)

  useEffect(() => { runObserver.current = onRunObserved }, [onRunObserved])
  useEffect(() => {
    mounted.current = true
    return () => { mounted.current = false }
  }, [])

  const save = useCallback((next: StoredConversation) => {
    storedRef.current = next
    if (mounted.current) setStored(next)
    try {
      const serialized = JSON.stringify(next)
      localStorage.setItem(`lingtu.agent.conversation:${location.origin}`, serialized)
      if (next.conversation_id) localStorage.setItem(`lingtu.agent.draft:${location.origin}:${next.conversation_id}`, serialized)
    }
    catch { if (mounted.current) setNotice('浏览器无法保存会话，刷新后可能无法自动恢复。') }
  }, [])

  const loadOptions = useCallback(async () => {
    if (optionsRequest.current) return optionsRequest.current
    const operation = async () => {
      if (mounted.current) setOptionsLoading(true)
      try {
        const next = await fetchConversationOptions()
        if (!mounted.current) return
        optionsRef.current = next
        setOptions(next)
        setOptionsError('')
        const current = storedRef.current
        const model = next.models.find(item => item.id === current.model && item.configured !== false)
          ?? next.models.find(item => item.id === next.default_model)!
        const reasoning = model.reasoning.find(item => item.id === current.reasoning)?.id
          ?? model.reasoning.find(item => item.id === next.default_reasoning)?.id ?? model.reasoning[0].id
        save({ ...current, model: model.id, reasoning })
      } catch (failure) {
        optionsRef.current = null
        if (mounted.current) {
          setOptions(null)
          setOptionsError(failure instanceof Error ? failure.message : String(failure))
        }
      } finally { if (mounted.current) setOptionsLoading(false) }
    }
    optionsRequest.current = operation()
    try { await optionsRequest.current } finally { optionsRequest.current = null }
  }, [save])

  useEffect(() => { if (enabled) void loadOptions() }, [loadOptions, enabled])

  const refresh = useCallback(async () => {
    if (refreshRef.current) return refreshRef.current
    const epoch = sessionEpoch.current
    const mutation = mutationEpoch.current
    const operation = async () => {
      if (mounted.current && !snapshotRef.current) setLoading(true)
      try {
        let id = storedRef.current.conversation_id
        if (!id) {
          id = await createConversation()
          if (!mounted.current || epoch !== sessionEpoch.current) return
          save({ ...storedRef.current, conversation_id: id })
        }
        let next: ConversationSnapshot
        try { next = await fetchConversation(id) }
        catch (failure) {
          if (!mounted.current || epoch !== sessionEpoch.current) return
          if (!(failure instanceof ConversationError) || failure.status !== 404) throw failure
          id = await createConversation()
          if (!mounted.current || epoch !== sessionEpoch.current) return
          save({ ...storedRef.current, conversation_id: id, pending: null, attachments: [], replyTo: '', replyQuote: '' })
          snapshotRef.current = null
          if (mounted.current) {
            setSnapshot(null)
            setNotice('原会话已不存在，历史未恢复。已建立新会话，保留的草稿需重新发送，附件需重新添加。')
          }
          next = await fetchConversation(id)
        }
        if (!mounted.current || epoch !== sessionEpoch.current || mutation !== mutationEpoch.current) return
        next = { ...next, messages: mergeConversationMessages(snapshotRef.current?.messages ?? [], next.messages),
          history_cursor: expandedHistory.current === id ? snapshotRef.current?.history_cursor : next.history_cursor }
        snapshotRef.current = next
        setSnapshot(next)
        const pending = storedRef.current.pending
        setRefreshError('')
        if (pending && next.messages.some(message => message.id === pending.message_id)) {
          save({ ...storedRef.current, pending: null, draft: '', replyTo: '', replyQuote: '', attachments: [] })
          setError('')
        }
        const runMessage = next.messages.findLast(message => Boolean(message.run_id))
        const runId = next.active_run?.run_id || runMessage?.run_id
          || next.turns.findLast(turn => Boolean(turn.run_id))?.run_id
        if (runId && !observedRuns.current.has(runId)) {
          observedRuns.current.add(runId)
          const turnId = runMessage?.turn_id
          const instruction = next.active_run?.instruction
            || next.messages.find(message => message.role === 'user' && message.turn_id === turnId)?.text || '当前任务'
          runObserver.current?.(runId, instruction)
        }
      } catch (failure) {
        if (mounted.current && epoch === sessionEpoch.current) setRefreshError(failure instanceof Error ? failure.message : String(failure))
      } finally { if (mounted.current && epoch === sessionEpoch.current) setLoading(false) }
    }
    const pendingRefresh = operation()
    refreshRef.current = pendingRefresh
    try { await pendingRefresh } finally { if (refreshRef.current === pendingRefresh) refreshRef.current = null }
  }, [save])

  const loadHistory = useCallback(async () => {
    if (historyRequest.current) return historyRequest.current
    const current = snapshotRef.current
    const cursor = current?.history_cursor
    if (!current || cursor == null) return false
    const epoch = sessionEpoch.current
    setHistoryLoading(true)
    const operation = (async () => {
      try {
        const page = await fetchConversation(current.conversation_id, cursor)
        if (!mounted.current || epoch !== sessionEpoch.current) return false
        const latest = snapshotRef.current!
        expandedHistory.current = current.conversation_id
        snapshotRef.current = { ...latest, messages: mergeConversationMessages(latest.messages, page.messages), history_cursor: page.history_cursor }
        setSnapshot(snapshotRef.current)
        return true
      } catch (failure) {
        if (mounted.current && epoch === sessionEpoch.current) setError(failure instanceof Error ? failure.message : String(failure))
        return false
      } finally { if (mounted.current && epoch === sessionEpoch.current) setHistoryLoading(false) }
    })()
    historyRequest.current = operation
    try { return await operation }
    finally {
      if (historyRequest.current === operation) historyRequest.current = null
    }
  }, [])

  useEffect(() => {
    if (!enabled) return
    const event = parseConversationMessageEvent(liveMessage)
    if (!event || event.conversation_id !== storedRef.current.conversation_id) return
    const current = snapshotRef.current ?? { conversation_id: event.conversation_id, messages: [], turns: [] }
    const previous = current.messages.find(message => message.id === event.id)
    if (previous && (previous.revision ?? 0) >= (event.revision ?? 0)) return
    const turn = current.turns.find(item => item.turn_id === event.turn_id)
    if (event.metadata?.status === 'streaming' && turn && turn.state !== 'running') return
    snapshotRef.current = { ...current, messages: mergeConversationMessages(current.messages, [event]) }
    setSnapshot(snapshotRef.current)
  }, [enabled, liveMessage])

  const active = snapshot?.turns.some(turn => turn.state === 'running') ?? false
  useEffect(() => {
    if (!enabled || !options) return
    if (!open && !storedRef.current.conversation_id && !active) return
    let disposed = false
    let timer: ReturnType<typeof setTimeout> | undefined
    const poll = async () => {
      await refresh()
      if (!disposed && (open || snapshotRef.current?.turns.some(turn => turn.state === 'running' || turn.state === 'queued'))) {
        timer = setTimeout(poll, 1_000)
      }
    }
    void poll()
    return () => { disposed = true; if (timer) clearTimeout(timer) }
  }, [open, active, snapshot?.conversation_id, refresh, enabled, options])

  const loadSessions = useCallback(async (archived = archivedView) => {
    const requestId = ++sessionsRequest.current
    setSessionsLoading(true)
    try {
      const items = await listConversations(archived)
      if (mounted.current && requestId === sessionsRequest.current) setConversations(items)
    } catch (failure) {
      if (mounted.current && requestId === sessionsRequest.current) setError(failure instanceof Error ? failure.message : String(failure))
    } finally { if (mounted.current && requestId === sessionsRequest.current) setSessionsLoading(false) }
  }, [archivedView])

  useEffect(() => { if (enabled && options && sessionListOpen) void loadSessions() }, [sessionListOpen, loadSessions, enabled, options])

  useEffect(() => {
    let disposed = false
    const query = searchQuery.trim()
    setSearchResults([])
    setSearchError('')
    setSearchBusy(Boolean(enabled && options && sessionListOpen && query))
    if (!enabled || !options || !sessionListOpen || !query) return
    const timer = setTimeout(async () => {
      try {
        const results = await searchConversations(query, archivedView)
        if (!disposed) setSearchResults(results)
      } catch (failure) {
        if (!disposed) setSearchError(failure instanceof Error ? failure.message : String(failure))
      } finally { if (!disposed) setSearchBusy(false) }
    }, 250)
    return () => { disposed = true; clearTimeout(timer) }
  }, [searchQuery, archivedView, sessionListOpen, enabled, options])

  const withSessionAction = useCallback(async (key: string, action: () => Promise<void>) => {
    if (sessionBusyRef.current || sendingRef.current || uploadingRef.current || approvalBusyRef.current || shareBusyRef.current) {
      const busy = new Error('请等待当前操作完成。')
      setError(busy.message)
      throw busy
    }
    sessionBusyRef.current = key
    setSessionBusy(key)
    setError('')
    try { await action() }
    catch (failure) {
      if (mounted.current) setError(failure instanceof Error ? failure.message : String(failure))
      throw failure
    } finally {
      sessionBusyRef.current = null
      if (mounted.current) setSessionBusy(null)
    }
  }, [])

  const selectSession = useCallback(async (id: string, fresh = false, initialSnapshot?: ConversationSnapshot) => {
    expandedHistory.current = null
    historyRequest.current = null
    setHistoryLoading(false)
    setSearchTarget(null)
    const current = storedRef.current
    const draft = readStoredConversation(`lingtu.agent.draft:${location.origin}:${id}`)
    const options = optionsRef.current
    const model = options?.models.find(item => item.id === (draft.model || current.model))
      ?? options?.models.find(item => item.id === options.default_model)
    const reasoning = model?.reasoning.find(item => item.id === (draft.reasoning || current.reasoning))?.id
      ?? model?.reasoning[0]?.id ?? ''
    sessionEpoch.current++
    refreshRef.current = null
    snapshotRef.current = null
    setSnapshot(null)
    setRefreshError('')
    setNotice('')
    save({ ...draft, conversation_id: id, model: model?.id ?? '', reasoning,
      ...(fresh && !current.conversation_id ? { draft: current.draft, attachments: current.attachments,
        includeMapContext: current.includeMapContext, includeSelection: current.includeSelection } : {}) })
    setOpen(true)
    if (initialSnapshot) {
      snapshotRef.current = initialSnapshot
      setSnapshot(initialSnapshot)
    } else await refresh()
  }, [refresh, save])

  const switchConversation = useCallback(async (id: string) => {
    await withSessionAction(id, async () => {
      if (id !== storedRef.current.conversation_id) {
        if (!storedRef.current.conversation_id && storedRef.current.draft.trim()) await refresh()
        await selectSession(id)
      }
      setOpen(true)
      setSessionListOpen(false)
    })
  }, [refresh, selectSession, withSessionAction])

  const openSearchResult = useCallback(async (result: ConversationSearchResult) => {
    const query = searchQuery.trim()
    setSearchTarget(null)
    await switchConversation(result.conversation_id)
    await withSessionAction('search', async () => {
      const epoch = sessionEpoch.current
      while (mounted.current && epoch === sessionEpoch.current && snapshotRef.current?.conversation_id === result.conversation_id) {
        if (snapshotRef.current.messages.some(message => message.id === result.message_id)) {
          setSearchTarget({ conversationId: result.conversation_id, messageId: result.message_id, query })
          return
        }
        if (snapshotRef.current.history_cursor == null) {
          setError('这条搜索结果已不存在，请重新搜索。')
          return
        }
        if (!await loadHistory()) return
      }
    })
  }, [switchConversation, loadHistory, searchQuery, withSessionAction])

  const createNewConversation = useCallback(async () => {
    await withSessionAction('create', async () => {
      if (refreshRef.current) await refreshRef.current
      const id = await createConversation()
      await selectSession(id, true)
      setSessionListOpen(false)
      await loadSessions()
    })
  }, [selectSession, loadSessions, withSessionAction])

  const forkConversation = useCallback(async (messageId: string) => {
    const current = storedRef.current
    const snapshot = snapshotRef.current
    const target = snapshot?.messages.find(message => message.id === messageId)
    const turn = snapshot?.turns.find(item => item.turn_id === target?.turn_id)
    if (current.pending) { setError('请先处理待重试消息，再创建会话分支。'); return }
    if (!current.conversation_id || !target || target.role !== 'assistant' || !turn
      || !['completed', 'failed', 'cancelled', 'interrupted'].includes(turn.state)
      || target.metadata?.status === 'streaming'
      || snapshot?.messages.some(message => message.turn_id === target.turn_id && message.role === 'assistant' && message.seq > target.seq)) {
      setError('只能从已结束轮次的最后一条回复创建分支。')
      return
    }
    await withSessionAction('fork', async () => {
      const metadata = await createConversationFork(current.conversation_id, messageId)
      let branch: ConversationSnapshot
      try { branch = await fetchConversation(metadata.conversation_id) }
      catch {
        throw new Error('分支已创建，但暂时无法读取。原会话已保留，请从历史会话打开分支，避免重复创建。')
      }
      if (!mounted.current) return
      setError('')
      await selectSession(metadata.conversation_id, false, { ...branch, metadata: branch.metadata ?? metadata })
      setSessionListOpen(false)
      setArchivedView(false)
      await loadSessions(false)
    })
  }, [selectSession, loadSessions, withSessionAction])

  const changeConversation = useCallback(async (id: string, changes: { title?: string; archived?: boolean }) => {
    await withSessionAction(id, async () => {
      const metadata = await updateConversation(id, changes)
      if (snapshotRef.current?.conversation_id === id) {
        sessionEpoch.current++
        refreshRef.current = null
        snapshotRef.current = { ...snapshotRef.current, metadata }
        setSnapshot(snapshotRef.current)
      }
      await loadSessions()
    })
  }, [loadSessions, withSessionAction])

  const renameConversation = useCallback((id: string, title: string) => changeConversation(id, { title }), [changeConversation])
  const archiveConversation = useCallback((id: string, archived: boolean) => changeConversation(id, { archived }), [changeConversation])

  const changeShare = useCallback(async (revoke: boolean): Promise<string | void> => {
    if (shareBusyRef.current || sessionBusyRef.current) return
    shareBusyRef.current = true
    setShareBusy(true)
    setError('')
    try {
      if (!storedRef.current.conversation_id) await refresh()
      const id = storedRef.current.conversation_id
      if (!id) throw new Error('会话尚未连接，请重试。')
      if (revoke) await deleteConversationShare(id)
      else await createConversationShare(id)
      if (refreshRef.current) await refreshRef.current
      await refresh()
      if (!revoke) {
        const share = snapshotRef.current?.share
        if (share) return `${location.origin}/?conversation-share=${encodeURIComponent(share.token)}`
      }
    } catch (failure) {
      if (mounted.current) setError(failure instanceof Error ? failure.message : String(failure))
      throw failure
    } finally {
      shareBusyRef.current = false
      if (mounted.current) setShareBusy(false)
    }
  }, [refresh])

  const shareCurrentConversation = useCallback(() => changeShare(false), [changeShare])
  const revokeShare = useCallback(async () => { await changeShare(true) }, [changeShare])
  const shareUrl = snapshot?.share ? `${location.origin}/?conversation-share=${encodeURIComponent(snapshot.share.token)}` : ''

  const setDraft = useCallback((draft: string) => {
    const current = storedRef.current
    if (!current.pending || current.editable) save({ ...current, draft,
      pending: draft !== current.draft ? null : current.pending })
  }, [save])

  const setIncludeMapContext = useCallback((includeMapContext: boolean) => {
    const current = storedRef.current
    if (current.pending || sendingRef.current || sessionBusyRef.current) return
    save({ ...current, includeMapContext })
  }, [save])

  const setIncludeSelection = useCallback((includeSelection: boolean) => {
    const current = storedRef.current
    if (current.pending || sendingRef.current || sessionBusyRef.current) return
    save({ ...current, includeSelection })
  }, [save])

  const setQueuedEdit = useCallback((queuedEdit: StoredConversation['queuedEdit']) => {
    if (storedRef.current.conversation_id !== stored.conversation_id) return
    save({ ...storedRef.current, queuedEdit })
  }, [save, stored.conversation_id])

  const setReplyTo = useCallback((replyTo: string, replyQuote = '') => {
    const current = storedRef.current
    if (!current.pending || current.editable) save({ ...current, replyTo, replyQuote: replyTo ? replyQuote : '', pending: null })
  }, [save])

  const setModel = useCallback((id: string) => {
    const current = storedRef.current
    const model = optionsRef.current?.models.find(item => item.id === id)
    if (!model || model.configured === false || sendingRef.current || uploadingRef.current || (current.pending && !current.editable)) return
    save({ ...current, model: id,
      reasoning: model.reasoning.find(item => item.id === optionsRef.current?.default_reasoning)?.id ?? model.reasoning[0].id,
      pending: null })
  }, [save])

  const setReasoning = useCallback((reasoning: string) => {
    const current = storedRef.current
    if (sendingRef.current || uploadingRef.current || (current.pending && !current.editable)
      || !optionsRef.current?.models.find(model => model.id === current.model)?.reasoning.some(item => item.id === reasoning)) return
    save({ ...current, reasoning, pending: null })
  }, [save])

  const setPermissionMode = useCallback((permissionMode: ConversationPermission) => {
    const current = storedRef.current
    if (!optionsRef.current?.permission_modes.includes(permissionMode) || sendingRef.current || uploadingRef.current
      || (current.pending && !current.editable)) return
    save({ ...current, permissionMode, pending: null })
  }, [save])

  const decideApproval = useCallback(async (approvalId: string, approved: boolean) => {
    const id = storedRef.current.conversation_id
    if (!id || approvalBusyRef.current || sessionBusyRef.current
      || !snapshotRef.current?.approvals?.some(item => item.approval_id === approvalId && item.state === 'pending')) return
    approvalBusyRef.current = approvalId
    setApprovalBusy(approvalId)
    setError('')
    try {
      await resolveConversationApproval(id, approvalId, approved)
      mutationEpoch.current++
    } catch (failure) {
      if (mounted.current) setError(failure instanceof Error ? failure.message : String(failure))
    } finally {
      if (refreshRef.current) await refreshRef.current
      await refresh()
      approvalBusyRef.current = null
      if (mounted.current) setApprovalBusy(null)
    }
  }, [refresh])

  const changeQueuedMessage = useCallback(async (turnId: string, text: string | null, promote = false) => {
    const id = storedRef.current.conversation_id
    try {
      if (promote) await promoteQueuedMessage(id, turnId)
      else await editQueuedMessage(id, turnId, text)
      mutationEpoch.current++
    } finally {
      if (refreshRef.current) await refreshRef.current
      await refresh()
    }
  }, [refresh])

  const addAttachments = useCallback(async (files: File[]) => {
    const current = storedRef.current
    const baseLimits = optionsRef.current?.attachments
    const model = optionsRef.current?.models.find(item => item.id === storedRef.current.model)
    const limits = baseLimits ? { ...baseLimits, accept: model?.attachment_accept ?? baseLimits.accept } : undefined
    if (!files.length || !limits || uploadingRef.current || sendingRef.current || sessionBusyRef.current || (current.pending && !current.editable)) return
    setOpen(true)
    setError('')
    if (current.attachments.length + files.length > limits.max_files) {
      setError(`每条消息最多添加 ${limits.max_files} 个附件。`)
      return
    }
    uploadingRef.current = true
    setUploading(true)
    try {
      if (!storedRef.current.conversation_id) await refresh()
      const id = storedRef.current.conversation_id
      if (!id) throw new Error('会话尚未连接，请刷新后重试。')
      for (const file of files) {
        const attachment = await uploadConversationAttachment(id, file, makeRequestId('attachment'), limits, storedRef.current.model)
        save({ ...storedRef.current, attachments: [...storedRef.current.attachments, attachment], pending: null })
      }
    } catch (failure) {
      if (mounted.current) setError(failure instanceof Error ? failure.message : String(failure))
    } finally {
      uploadingRef.current = false
      if (mounted.current) setUploading(false)
    }
  }, [refresh, save])

  const removeAttachment = useCallback((id: string) => {
    const current = storedRef.current
    if (sendingRef.current || uploadingRef.current || (current.pending && !current.editable)) return
    save({ ...current, attachments: current.attachments.filter(item => item.id !== id), pending: null })
  }, [save])

  const send = useCallback(async (regenerateMessageId?: string, reply?: { text: string; messageId: string }) => {
    if (reply && (storedRef.current.pending || storedRef.current.draft.trim() || storedRef.current.replyTo || storedRef.current.attachments.length)) return
    const snapshot = snapshotRef.current
    const latestReply = snapshot?.messages.filter(message => message.role === 'assistant').at(-1)
    if (regenerateMessageId && (latestReply?.id !== regenerateMessageId || storedRef.current.pending
      || storedRef.current.draft.trim() || storedRef.current.replyTo || storedRef.current.attachments.length
      || snapshot?.turns.some(turn => turn.state === 'running') || snapshot?.active_run
      || snapshot?.approvals?.some(approval => approval.state === 'pending'))) return
    if (sendingRef.current || uploadingRef.current || sessionBusyRef.current || !optionsRef.current
      || (!regenerateMessageId && !reply && !storedRef.current.draft.trim() && !storedRef.current.attachments.length && !storedRef.current.pending)) return
    if (snapshotRef.current?.metadata?.archived) { setError('此会话已归档，恢复后可以继续对话。'); return }
    const current = storedRef.current
    const selected = optionsRef.current.models.find(item => item.id === current.model)
    if (!current.pending && selected && !selected.image_input
      && current.attachments.some(attachment => attachment.mime_type.startsWith('image/'))) {
      setOpen(true)
      setError('当前模型不支持图片，请切换支持图片的模型或移除图片附件。')
      return
    }
    sendingRef.current = true
    setSending(true)
    setError('')
    setOpen(true)
    try {
      if (!storedRef.current.conversation_id) await refresh()
      const current = storedRef.current
      if (!current.conversation_id) return
      const persona = botPersona(readBotProfile())
      const replyId = regenerateMessageId ?? reply?.messageId ?? current.replyTo
      // Regeneration is a new read-only turn; retrying its receipt keeps the same request ID.
      const pending = current.pending ?? { enqueue: true, message_id: makeRequestId('message'), text: regenerateMessageId
        ? '请基于当前对话重新生成上一条回答。仅重新组织回答；如需补充，只查询信息，不重复执行任何动作。' : reply?.text ?? current.draft.trim(),
        ...(replyId ? { reply_to_message_id: replyId } : {}),
        ...(!regenerateMessageId && !reply && current.replyQuote ? { reply_quote: current.replyQuote } : {}),
        ...(persona ? { persona } : {}),
        model: current.model, reasoning: current.reasoning, permission_mode: regenerateMessageId ? 'read_only' as const : current.permissionMode, map_context: mapContext.current,
        attachment_ids: current.attachments.map(item => item.id) }
      save({ ...current, pending, editable: false })
      const turn = await sendConversationMessage(current.conversation_id, pending)
      mutationEpoch.current++
      if (!mounted.current) return
      const next = snapshotRef.current ?? { conversation_id: current.conversation_id, messages: [], turns: [] }
      snapshotRef.current = { ...next, turns: [...next.turns.filter(item => item.turn_id !== turn.turn_id), turn] }
      setSnapshot(snapshotRef.current)
      save({ ...storedRef.current, pending: null, draft: '', replyTo: '', replyQuote: '', attachments: [] })
      if (refreshRef.current) await refreshRef.current
      await refresh()
    } catch (failure) {
      if (failure instanceof ConversationError && failure.status >= 400 && failure.status < 500) {
        save({ ...storedRef.current, editable: true })
      }
      if (mounted.current) setError(`${failure instanceof Error ? failure.message : String(failure)} 原消息和请求 ID 已保留，可重试确认。`)
    } finally {
      sendingRef.current = false
      if (mounted.current) setSending(false)
    }
  }, [refresh, save])

  const reconnect = useCallback(async () => {
    await loadOptions()
    if (optionsRef.current) await refresh()
  }, [loadOptions, refresh])

  return { open, setOpen, draft: stored.draft, setDraft, replyTo: stored.replyTo, replyQuote: stored.replyQuote, setReplyTo, pending: stored.pending, editable: !stored.pending || Boolean(stored.editable), snapshot,
    sessionListOpen, setSessionListOpen, conversations, archivedView, setArchivedView, sessionsLoading, sessionBusy,
    searchQuery, setSearchQuery, searchResults, searchBusy, searchError, openSearchResult, searchTarget,
    context, includeMapContext: stored.includeMapContext, includeSelection: stored.includeSelection, setIncludeMapContext, setIncludeSelection,
    createNewConversation, switchConversation, renameConversation, archiveConversation, forkConversation,
    shareUrl, shareBusy, shareCurrentConversation, revokeShare,
    model: stored.model, reasoning: stored.reasoning, setModel, setReasoning, attachments: stored.attachments,
    permissionMode: stored.permissionMode, setPermissionMode, decideApproval, approvalBusy,
    addAttachments, removeAttachment, uploading, options, optionsLoading, optionsError,
    answerQuestion: (messageId: string, text: string) => send(undefined, { messageId, text }),
    queuedEdit: stored.queuedEdit ?? null, setQueuedEdit, loadHistory, historyLoading,
    changeQueuedMessage, sending, loading, active, error: error || refreshError, notice, send, refresh, reconnect }
}
