import type { MapEditOperation, MapEditSource } from '../types/mapEdit.ts'
import { validMapEditOperation } from './mapEdit.ts'

export interface MapEditDraft {
  source: MapEditSource
  name: string
  history: { edits: MapEditOperation[]; cursor: number }
  status: 'editing' | 'unconfirmed'
}

export type MapDraftStorage = Pick<Storage, 'getItem' | 'setItem' | 'removeItem'>
export type MapDraftResult = { kind: 'ready' | 'stale' | 'unconfirmed'; draft: MapEditDraft }

const keyFor = (source: MapEditSource) => `lingtu.map-edit-draft.v1:${source.name}`

function isRecord(value: unknown): value is Record<string, unknown> {
  return value !== null && typeof value === 'object' && !Array.isArray(value)
}

function isDraft(value: unknown): value is MapEditDraft {
  if (!isRecord(value) || !isRecord(value.source) || !isRecord(value.history)) return false
  const { source, history } = value
  if (typeof source.name !== 'string' || !source.name
    || !Number.isInteger(source.contentEpoch) || (source.contentEpoch as number) < 0
    || typeof source.frameId !== 'string' || !source.frameId
    || typeof value.name !== 'string'
    || (value.status !== 'editing' && value.status !== 'unconfirmed')
    || !Array.isArray(history.edits) || history.edits.length === 0
    || !Number.isInteger(history.cursor) || (history.cursor as number) < 0
    || (history.cursor as number) > history.edits.length) return false
  return history.edits.every(validMapEditOperation)
}

export function readMapDraft(source: MapEditSource, storage?: MapDraftStorage): MapDraftResult | null {
  try {
    const raw = (storage ?? window.localStorage).getItem(keyFor(source))
    if (raw === null) return null
    const draft: unknown = JSON.parse(raw)
    if (!isDraft(draft) || draft.source.name !== source.name) return null
    const kind = draft.status === 'unconfirmed' ? 'unconfirmed'
      : draft.source.contentEpoch === source.contentEpoch && draft.source.frameId === source.frameId
        ? 'ready' : 'stale'
    return { kind, draft }
  } catch {
    return null
  }
}

export function writeMapDraft(draft: MapEditDraft, storage?: MapDraftStorage): boolean {
  try {
    if (!isDraft(draft)) return false
    const record: MapEditDraft = {
      source: { name: draft.source.name, contentEpoch: draft.source.contentEpoch, frameId: draft.source.frameId },
      name: draft.name,
      history: {
        edits: structuredClone(draft.history.edits),
        cursor: draft.history.cursor,
      },
      status: draft.status,
    }
    ;(storage ?? window.localStorage).setItem(keyFor(draft.source), JSON.stringify(record))
    return true
  } catch {
    return false
  }
}

export function removeMapDraft(source: MapEditSource, storage?: MapDraftStorage): boolean {
  try {
    ;(storage ?? window.localStorage).removeItem(keyFor(source))
    return true
  } catch {
    return false
  }
}
