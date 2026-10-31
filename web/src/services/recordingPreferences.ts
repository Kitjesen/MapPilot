const KEY = 'lingtu.recording.selection'

interface RecordingPreferences {
  topics: string[]
  camera: boolean
}

export function readRecordingPreferences(storage?: Pick<Storage, 'getItem'>): RecordingPreferences | null {
  try {
    const value: unknown = JSON.parse((storage ?? window.localStorage).getItem(KEY) ?? 'null')
    if (value === null || typeof value !== 'object' || !('topics' in value)
      || !Array.isArray(value.topics) || !value.topics.every(topic => typeof topic === 'string')) return null
    return { topics: value.topics, camera: 'camera' in value && value.camera === true }
  } catch { return null }
}

export function writeRecordingPreferences(preferences: RecordingPreferences, storage?: Pick<Storage, 'setItem'>): void {
  try { (storage ?? window.localStorage).setItem(KEY, JSON.stringify(preferences)) } catch { /* Keep selection usable when browser storage is unavailable. */ }
}
