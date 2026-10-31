export interface RecordingNumericField { path: string; unit?: string }
export interface RecordingNumericSample { t: number; value: number | null; gap_before?: boolean }
export interface RecordingSeries {
  version: number
  ok: boolean
  mode: 'series'
  topic: string
  field: string | null
  fields: RecordingNumericField[]
  start_time_ns: string
  duration_s: number
  window: { start_s: number; end_s: number }
  samples: RecordingNumericSample[]
  original_count: number
  read_count: number
  numeric_count: number
  returned_count: number
  downsampling: { method: 'none' | 'minmax'; max_points: number }
  gap_threshold_s?: number
  frame_id: string | null
}
export interface RecordingSeriesResponse { ok: boolean; session_id: string; series: RecordingSeries }
export interface RecordingSeriesQuery {
  topic: string
  field?: string
  start?: number
  end?: number
  max_points?: number
}
