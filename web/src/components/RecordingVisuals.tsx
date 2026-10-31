import { useEffect, useRef, useState } from 'react'
import { Camera, Layers3, Radio, RefreshCw } from 'lucide-react'
import { text, type Locale } from '../i18n'
import { recordingCloudView, sceneFrameAt, sceneFrames } from '../services/recordingScene'
import { createRecordingCameraLoader, recordingCameraCoverage, recordingCameraTarget, type RecordingCameraLoadState } from '../services/recordingCamera'
import { useRecordingSceneWindow } from '../hooks/useRecordingScene'
import type { RecordingCameraFrame, RecordingSceneInfoResponse } from '../types/recordingScene'
import { PointCloudViewer } from './PointCloudViewer'
import styles from './RecordingVisuals.module.css'

function RecordedCamera({ sessionId, time, cameraTime, playing, info, locale }: { sessionId: string; time: number; cameraTime: number; playing: boolean; info: RecordingSceneInfoResponse | undefined; locale: Locale }) {
  const [chosenCamera, setChosenCamera] = useState('')
  const cameras = [...new Set(info?.camera.segments.map(segment => segment.camera_id) ?? [])]
  const camera = cameras.includes(chosenCamera) ? chosenCamera : cameras[0]
  const coverage = recordingCameraCoverage(info?.camera.segments ?? [], time, camera)
  const segment = coverage.kind === 'covered' ? coverage.segment : undefined
  const target = recordingCameraTarget(sessionId, coverage, time, cameraTime, playing)
  const [image, setImage] = useState<{ url: string; sessionId: string; segment: number; camera: string; frame: RecordingCameraFrame } | null>(null)
  const [loadState, setLoadState] = useState<RecordingCameraLoadState>({ phase: 'idle' })
  const urls = useRef<string[]>([])
  const loaderRef = useRef<ReturnType<typeof createRecordingCameraLoader> | null>(null)
  useEffect(() => {
    const loader = createRecordingCameraLoader({
      onState: setLoadState,
      onFrame: (request, frame) => {
        if (!frame) { setImage(null); return }
        const url = URL.createObjectURL(frame.blob)
        urls.current.push(url)
        while (urls.current.length > 3) URL.revokeObjectURL(urls.current.shift()!)
        setImage({ url, sessionId: request.sessionId, segment: request.segment, camera: request.cameraId, frame })
      },
    })
    loaderRef.current = loader
    return () => {
      loader.dispose()
      loaderRef.current = null
      for (const url of urls.current) URL.revokeObjectURL(url)
      urls.current = []
    }
  }, [sessionId])
  const targetTime = target?.time
  const targetCamera = target?.cameraId
  const targetSegment = target?.segment
  useEffect(() => {
    loaderRef.current?.update(targetTime === undefined || targetSegment === undefined || !targetCamera ? null : {
      sessionId, cameraId: targetCamera, segment: targetSegment, time: targetTime, cursor: time, playing,
    })
  }, [sessionId, targetTime, targetCamera, targetSegment, time, playing])
  const matches = !!image && image.sessionId === sessionId && image.camera === camera && image.segment === segment?.index
  const visible = !!segment && matches && image.frame.t <= time && image.frame.t >= segment.start_s
  const frame = matches ? image.frame : undefined
  const exact = (frame?.timing ?? segment?.timing) === 'frame_timestamp_index'
  const frameId = frame?.frameId || segment?.frame_id
  const state = coverage.kind !== 'covered' ? coverage.kind : loadState.phase === 'error' ? 'error'
    : loadState.phase === 'empty' ? 'empty' : visible ? loadState.phase === 'loading' ? 'updating' : 'ready' : 'loading'
  const retry = () => loaderRef.current?.retry()
  const explanation = coverage.kind === 'before' ? text(locale, `Camera starts at ${coverage.nextStart.toFixed(2)} s`, `相机从 ${coverage.nextStart.toFixed(2)} s 开始`)
    : coverage.kind === 'gap' ? text(locale, `No recorded image here. Next segment starts at ${coverage.nextStart.toFixed(2)} s.`, `此时段没有录制画面，下一段从 ${coverage.nextStart.toFixed(2)} s 开始。`)
    : coverage.kind === 'ended' ? text(locale, `Camera recording has ended. Coverage ends at ${coverage.end.toFixed(2)} s.`, `相机片段已结束，覆盖到 ${coverage.end.toFixed(2)} s。`)
    : coverage.kind === 'unrecorded' ? text(locale, 'Camera was not recorded', '未录制相机')
    : state === 'error' ? text(locale, 'Camera frame could not be read', '相机帧读取失败')
    : state === 'empty' ? text(locale, 'No camera frame was returned for this position', '此位置没有返回相机帧')
    : text(locale, 'Reading the first camera frame…', '正在读取首帧…')
  return <section className={styles.media} aria-label={text(locale, 'Recorded camera', '录制相机')}>
    <div className={styles.mediaHeading}><span><Camera size={13} />{text(locale, 'Camera', '相机')}</span>{cameras.length > 1 && <select aria-label={text(locale, 'Recorded camera source', '录制相机来源')} value={camera} onChange={event => setChosenCamera(event.target.value)}>{cameras.map(value => <option key={value} value={value}>{value}</option>)}</select>}</div>
    <div className={styles.cameraViewport} data-camera-state={state} aria-busy={state === 'loading' || state === 'updating'}>
      {visible ? <img src={image.url} alt={text(locale, 'Recorded robot camera at the shared playhead', '共同播放位置对应的机器人录制画面')} onError={() => { setImage(null); setLoadState({ phase: 'error', error: 'recording_camera_image_decode_failed' }) }} /> : <div className={styles.empty}><Camera size={22} strokeWidth={1.3} /><span role="status">{explanation}</span>{(state === 'error' || state === 'empty') && <button className={styles.cameraRetry} onClick={retry}><RefreshCw size={13} />{text(locale, 'Retry', '重试')}</button>}</div>}
      {visible && (state === 'updating' || state === 'error') && <div className={styles.cameraStatus} role="status"><span>{state === 'updating' ? text(locale, 'Updating · showing the previous frame', '正在更新 · 显示上一帧') : text(locale, 'Update failed · showing the previous frame', '更新失败 · 显示上一帧')}</span>{state === 'error' && <button className={styles.cameraRetry} onClick={retry}><RefreshCw size={12} />{text(locale, 'Retry', '重试')}</button>}</div>}
    </div>
    {visible && <div className={styles.mediaMeta} data-gap={time - image.frame.t > 2 || undefined}><span>{image.frame.frameIndex !== null ? `#${image.frame.frameIndex} · ` : ''}{image.frame.t.toFixed(2)} s · {text(locale, 'age', '距播放位置')} {(time - image.frame.t).toFixed(2)} s</span><span>{exact ? text(locale, 'Indexed source frame', '原始帧时间索引') : text(locale, 'Estimated segment timing', '分段时间估计')} · {text(locale, 'preview ≤5 fps', '预览 ≤5 帧/秒')}</span></div>}
    {segment && <details className={styles.sourceDetails}><summary>{text(locale, 'Camera timing', '相机时间依据')}</summary><p>{exact || segment.exact ? text(locale, 'Source frame timestamps align with this recording clock. This does not establish sensor hardware synchronization.', '按录制的原始帧时间戳对齐；这不代表传感器硬件已同步。') : text(locale, 'Legacy recording: frame time is estimated from segment bounds and frame order, not measured per-frame synchronization.', '旧录制按分段起止和帧序估算时间，不是逐帧测得的精确同步。')}</p><code>{camera} · {frameId || text(locale, 'frame not recorded', '未记录坐标系')}{frame?.mediaTime !== null && frame?.mediaTime !== undefined ? ` · PTS ${frame.mediaTime.toFixed(3)} s` : ''}{frame?.sourceTimeNs ? ` · Unix ns ${frame.sourceTimeNs}` : ''}</code></details>}
    {state === 'error' && loadState.error && <p className={styles.error} role="status">{loadState.error}</p>}
  </section>
}
function RecordedScene({ sessionId, time, info, locale }: { sessionId: string; time: number; info: RecordingSceneInfoResponse | undefined; locale: Locale }) {
  const [cloudTopic, setCloudTopic] = useState('')
  const [poseTopic, setPoseTopic] = useState('')
  const [chosenFrame, setChosenFrame] = useState('')
  const topics = info?.scene?.topics ?? []
  const clouds = topics.filter(topic => topic.kind === 'cloud')
  const poses = topics.filter(topic => topic.kind === 'pose')
  const query = { cloud_topic: cloudTopic || undefined, pose_topic: poseTopic || undefined }
  const result = useRecordingSceneWindow(sessionId, time, info, query)
  const covered = time >= 0 && time <= (info?.scene?.duration_s ?? -1)
  const scene = covered ? result.scene : undefined
  const frames = sceneFrames(scene, time)
  const frameId = frames.includes(chosenFrame) ? chosenFrame : frames[0] ?? ''
  const cloud = sceneFrameAt(scene?.cloud?.frames ?? [], time)
  const pose = sceneFrameAt(scene?.trajectory?.samples ?? [], time)
  const view = recordingCloudView(scene, time, frameId)
  const hasGeometry = view.points.length > 0 || view.lines.some(line => line.points.length > 0)
  const age = cloud ? time - cloud.t : null
  const missing = cloud && (scene?.cloud?.frames.find(item => item.t > time)?.gap_before || age! > 2)
  return <section className={styles.media} aria-label={text(locale, 'Recorded 3D scene', '录制三维场景')}>
    <div className={styles.mediaHeading}><span><Layers3 size={13} />{text(locale, 'Cloud and trajectory', '点云与轨迹')}</span>{frames.length > 1 ? <select aria-label={text(locale, 'Recorded coordinate frame', '录制坐标系')} value={frameId} onChange={event => setChosenFrame(event.target.value)}>{frames.map(value => <option key={value} value={value}>{value || text(locale, 'Unknown', '未记录')}</option>)}</select> : <code>{frameId || '—'}</code>}</div>
    {(clouds.length > 1 || poses.length > 1) && <div className={styles.topicControls}>{clouds.length > 1 && <select aria-label={text(locale, 'Recorded point cloud topic', '录制点云数据流')} value={cloudTopic} onChange={event => setCloudTopic(event.target.value)}><option value="">{text(locale, 'Default cloud', '默认点云')}</option>{clouds.map(topic => <option key={topic.topic} value={topic.topic}>{topic.topic}</option>)}</select>}{poses.length > 1 && <select aria-label={text(locale, 'Recorded trajectory topic', '录制轨迹数据流')} value={poseTopic} onChange={event => setPoseTopic(event.target.value)}><option value="">{text(locale, 'Default trajectory', '默认轨迹')}</option>{poses.map(topic => <option key={topic.topic} value={topic.topic}>{topic.topic}</option>)}</select>}</div>}
    <div className={styles.sceneViewport}>{hasGeometry ? <PointCloudViewer key={frameId} mapName={null} recording={view} /> : <div className={styles.empty}><Radio size={22} strokeWidth={1.3} /><span>{result.error ? text(locale, 'Recorded scene unavailable', '三维数据暂不可用') : topics.length && covered && result.scene === undefined ? text(locale, 'Reading scene window…', '正在读取场景时间窗…') : text(locale, 'No 3D samples at this time', '此时间没有三维样本')}</span></div>}</div>
    <div className={styles.mediaMeta} data-gap={missing || undefined}><span>{cloud ? `${text(locale, 'Cloud', '点云')} ${cloud.t.toFixed(2)} s · ${age!.toFixed(2)} s ${text(locale, 'old', '前')}` : text(locale, 'No point cloud frame', '无点云帧')}{missing ? text(locale, ' · data gap / retained frame', ' · 数据缺口 / 保留旧帧') : ''}</span>{pose && <span>{text(locale, 'Pose', '位姿')} {pose.t.toFixed(2)} s</span>}</div>
    <div className={styles.sceneLegend}><span><i />{text(locale, 'Past trajectory in this window', '窗口内已行轨迹')}</span><span><i data-path />{text(locale, 'Recorded planned path', '记录的规划路径')}</span><span>{text(locale, 'Display sampling · ≤32 frames/window, ≤2,048 points/frame', '显示采样 · 每窗 ≤32 帧，每帧 ≤2,048 点')}</span></div>
    {frames.length > 1 && <p className={styles.note}>{text(locale, 'Coordinate frames are viewed separately; no transform is invented.', '坐标系分别查看，未推测或补造坐标变换。')}</p>}
    {frames.includes('') && <p className={styles.note}>{text(locale, 'Unknown frame: points are shown without overlaying trajectory or paths.', '坐标系未记录时仅显示点云，不叠加轨迹和规划路径。')}</p>}
    {result.error && <p className={styles.error} role="status">{result.error}</p>}
  </section>
}

export function RecordingVisualSource({ label, sessionId, time, cameraTime, playing, info, error, locale, single = false }: { label: string; sessionId: string; time: number; cameraTime: number; playing: boolean; info?: RecordingSceneInfoResponse; error?: string; locale: Locale; single?: boolean }) {
  return <article className={styles.source} data-single={single && !!info?.camera.available && !!info?.scene?.topics.length}>
    <header className={styles.sourceHeading}><strong>{label} · {sessionId}</strong><span>{time.toFixed(2)} s</span></header>
    {error && <p className={styles.error} role="status">{error}</p>}
    {!info ? <p className={styles.note}>{error ? text(locale, 'Recording contents unavailable', '录制内容暂不可用') : text(locale, 'Reading recording contents…', '正在读取录制内容…')}</p> : <div className={styles.mediaGrid}>
    {info?.camera.available ? <RecordedCamera sessionId={sessionId} time={time} cameraTime={cameraTime} playing={playing} info={info} locale={locale} /> : <p className={styles.note}>{text(locale, 'Camera was not recorded', '未录制相机')}</p>}
    {info?.scene?.topics.length ? <RecordedScene sessionId={sessionId} time={time} info={info} locale={locale} /> : <p className={styles.note}>{text(locale, 'No point cloud or trajectory was recorded', '未录制点云或轨迹')}</p>}
    </div>}
  </article>
}
