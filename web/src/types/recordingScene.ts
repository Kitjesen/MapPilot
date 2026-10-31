export interface RecordingSceneTopic { topic: string; type_name: string; kind: 'cloud' | 'pose' | 'path'; message_count: number }
export interface RecordingSceneStamp { t: number; source_time_ns: string; frame_id: string; gap_before?: boolean }
export interface RecordingCloudFrame extends RecordingSceneStamp { original_points: number; points: number[] }
export interface RecordingPoseSample extends RecordingSceneStamp { position: [number, number, number]; orientation: [number, number, number, number] }
export interface RecordingPathFrame extends RecordingSceneStamp { original_poses: number; points: number[] }
export interface RecordingScene {
  version: number
  ok: boolean
  mode: 'scene' | 'scene-info'
  start_time_ns: string
  duration_s: number
  window?: { start_s: number; end_s: number }
  topics: RecordingSceneTopic[]
  cloud: { topic: string; original_count: number; read_count: number; returned_count: number; frames: RecordingCloudFrame[] } | null
  trajectory: { topic: string; original_count: number; read_count: number; samples: RecordingPoseSample[] } | null
  paths: Array<{ topic: string; read_count: number; frames: RecordingPathFrame[] }>
}
export interface RecordingCameraSegment {
  index: number
  camera_id: string
  relative_path: string
  codec?: string
  width?: number
  height?: number
  start_time_ns: string
  end_time_ns: string
  start_s: number
  end_s: number
  coverage_end_s?: number
  frame_count: number
  dropped_frames: number
  frame_id?: string
  frame_url: string
  timing: 'frame_timestamp_index' | 'segment_cfr_estimate'
  exact: boolean
  frame_timestamps_ns?: string[]
}
export interface RecordingSceneInfoResponse {
  ok: boolean
  session_id: string
  state: string
  scene: RecordingScene | null
  start_time_ns: string
  duration_s: number
  camera: { available: boolean; timing: 'frame_timestamp_index' | 'segment_cfr_estimate'; exact: boolean; segments: RecordingCameraSegment[] }
}
export interface RecordingSceneResponse { ok: boolean; session_id: string; scene: RecordingScene | null }
export interface RecordingSceneQuery { start: number; end: number; cloud_topic?: string; pose_topic?: string; path_topic?: string[] }
export interface RecordingCameraFrame { blob: Blob; t: number; mediaTime: number | null; timing: string; sourceTimeNs: string | null; frameIndex: number | null; frameId: string | null }

export interface RecordingCloudView {
  points: Float32Array
  frameId: string
  originalCount: number | null
  lines: Array<{ points: Float32Array; kind: 'trajectory' | 'path' }>
}
