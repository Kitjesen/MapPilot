import { createRoot } from 'react-dom/client'
import { ObservationCard } from '../src/components/agentConversation/ObservationCard'
import { LiveCameraCard } from '../src/components/agentConversation/LiveCameraCard'
import type { ConversationSnapshot } from '../src/services/agentConversation'
import { installPreviewCamera, previewCamera } from './fixtures/previewCamera'
import '../src/App.css'
// Isolated origin, real components, fixture transport only. Never forward a request.
const id = 'style-preview'
installPreviewCamera()
Object.assign(window, { __previewCamera: previewCamera })
let generation = 0
const snapshot: ConversationSnapshot = { conversation_id: id, messages: [], turns: [], approvals: [], history_cursor: null }
function addMessage(turnId: string, role: 'user' | 'assistant' | 'tool', text: string, kind?: string) {
  const message = { id: `preview-${snapshot.messages.length + 1}`, seq: snapshot.messages.length + 1,
    turn_id: turnId, role, text, kind, created_at: Date.now() / 1000 }
  snapshot.messages.push(message)
  return message
}
const maps = { maps: [
  { name: '园区东区', size_mb: 32.6, can_activate: true, record: { state: 'READY', content_epoch: 12 } },
  { name: '办公区一层', size_mb: 18.2, can_activate: false, record: { state: 'STALE', content_epoch: 5 }, activation_blockers: ['需要重新生成导航数据'] },
] }
const mapPoints: number[][] = []
for (let x = 0; x <= 80; x++) for (let y = 0; y <= 50; y++) {
  mapPoints.push([x / 10, y / 10, 0])
  if (x === 0 || x === 80 || y === 0 || y === 50 || x === 40 && (y < 18 || y > 28)) {
    for (let z = 1; z <= 24; z++) mapPoints.push([x / 10, y / 10, z / 10])
  }
}
window.fetch = async (input, init) => {
  const path = new URL(typeof input === 'string' ? input : input instanceof URL ? input.href : input.url, location.origin).pathname
  const method = init?.method ?? (input instanceof Request ? input.method : 'GET')
  const body = init?.body ? JSON.parse(String(init.body)) : {}
  const reply = (value: unknown, status = 200) => new Response(JSON.stringify(value), { status, headers: { 'Content-Type': 'application/json' } })
  if (path === '/api/v1/state') return reply({ schema_version:4, ts:Date.now()/1000, session:{product:'nav'}, navigation:{schema_version:3, ts:Date.now()/1000, task:{state:'PAUSED',task_id:'demo-task',reason:'operator_pause'}, goal_admission:{state:'UNKNOWN',reason:''}, control:{authority:'NONE',resume_required:true,reason:''}, motion:{permission:'BLOCKED',observation:'QUIET',stop_confirmation:'CONFIRMED',reason:'operator_pause'}} })
  if (path.startsWith('/api/v1/robot/sensors/')) {
    const sensor = path.split('/').at(-1)
    return reply({ success:true, sensor, fresh:true, source_stamp_s:Date.now()/1000, values: sensor === 'joints' ? { joints:Array.from({length:16},(_,id)=>({id,position_rad:Math.sin(Date.now()/1000+id)*0.1,velocity_rad_s:0,torque_nm:0})) } : {motors:Array.from({length:16},(_,id)=>({id,online:id!==3,temperature_c:32+id,status_code:id===3?1:0,errors:id===3?['演示：通信中断']:[]}))} })
  }
  if (path === '/api/v1/robot/imu/live') return reply({state:'live',provider:'preview',source_clock:'demo',sequence:1,samples:[{seq:1,t:0,received_at_s:Date.now()/1000,values:{angular_velocity_rad_s:{x:0,y:0,z:0},linear_acceleration_m_s2:{x:0,y:0,z:9.81}}}],recording:null})
  if (path === '/api/v1/robot/imu/records' && method === 'GET') return reply({records:[]})
  if (path === '/api/v1/camera/status') return reply({ status: 'streaming', available: true })
  if (path === '/api/v1/webrtc/status') return reply({ available: false })
  if (path.startsWith('/api/v1/maps/') && path.endsWith('/preview')) {
    const name = decodeURIComponent(path.split('/')[4])
    const map = maps.maps.find(item => item.name === name)
    if (!map) return reply({ error: 'map_not_found' }, 404)
    return reply({ name, content_epoch: map.record.content_epoch, frame_id: 'map', points: mapPoints,
      count: mapPoints.length, total_points: mapPoints.length * 5, cleanup_state: 'unknown', built_at_ms: null })
  }
  if (path === '/api/v1/agent/options') return reply({ available: true, provider: 'preview', default_model: 'preview-model', default_reasoning: 'default', default_permission_mode: 'default', permission_modes: ['read_only', 'default', 'full_access'],
    models: [{ id: 'preview-model', label: '预览模型', configured: true, reasoning: [{ id: 'default', label: '默认' }], image_input: false }], attachments: { accept: '.txt', max_files: 4, max_file_bytes: 1000 } })
  if (path === '/api/v1/agent/conversations' && method === 'POST') return reply({ conversation_id: id }, 201)
  if (path === '/api/v1/agent/conversations') return reply({ conversations: [] })
  if (path.endsWith('/messages') && method === 'POST') {
    const turnId = `turn-${++generation}`
    const turn: ConversationSnapshot['turns'][number] = { turn_id: turnId, message_id: body.message_id, state: 'running', started_at: Date.now() / 1000 }
    snapshot.turns.push(turn)
    const user = addMessage(turnId, 'user', body.text); user.id = body.message_id
    if (body.text.includes('审批')) {
      snapshot.approvals!.push({ approval_id: `approval-${generation}`, turn_id: turnId, tool_name: 'save_map_copy', arguments: { map_id: '园区东区', name: '东区_检查后', operations: [] }, state: 'pending' })
    } else {
      const viewing = body.text.includes('三维')
      const camera = body.text.includes('相机')
      const observation = ({ '机器人状态': 'robot', '关节与故障': 'joints', 'IMU 与记录': 'imu', '导航任务': 'navigation' } as Record<string,string>)[body.text]
      const tool = addMessage(turnId, 'tool', '{}', observation ? 'view_robot_card' : camera ? 'view_live_camera' : viewing ? 'view_saved_map' : 'list_saved_maps')
      Object.assign(tool, { metadata: { status: 'running', started_at: Date.now() / 1000 } })
      setTimeout(() => {
        tool.text = JSON.stringify(observation ? { success: true, observation_view: observation } : camera ? { success: true, read_only: true, camera_view: { source: 'robot' } } : viewing ? { success: true, read_only: true, map_view: { map_id: '园区东区', content_epoch: 12 } } : maps)
        Object.assign(tool, { metadata: { status: 'completed', duration_ms: 850 } })
        turn.state = 'completed'; turn.finished_at = Date.now() / 1000
        addMessage(turnId, 'assistant', camera ? '点击卡片展开观看，收起即释放连接。' : viewing ? '下面是只读三维预览，可拖动旋转和缩放。' : '东区地图已就绪。办公区一层需要重新生成导航数据。', 'reply')
      }, 850)
    }
    return reply({ ...turn, conversation_id: id }, 202)
  }
  if (path.includes('/approvals/') && method === 'POST') {
    const approval = snapshot.approvals!.find(a => path.endsWith(a.approval_id))
    if (!approval) return reply({ detail: { reason: 'approval_not_found' } }, 404)
    await new Promise(resolve => setTimeout(resolve, 550))
    approval.state = body.approved ? 'approved' : 'denied'
    const turn = snapshot.turns.find(t => t.turn_id === approval.turn_id)!
    turn.state = 'completed'; turn.finished_at = Date.now() / 1000
    addMessage(turn.turn_id, 'assistant', body.approved ? '已记录允许。本页是交互演示，没有执行地图保存。' : '已拒绝，没有执行地图保存。', 'reply')
    return reply(approval)
  }
  if (path.startsWith('/api/v1/agent/conversations/') && method === 'GET') return reply({ ...snapshot, server_time: Date.now() / 1000 })
  return reply({ detail: { reason: 'preview_only' } }, 503)
}
createRoot(document.getElementById('root')!).render(<main style={{maxWidth:620,margin:'32px auto',padding:16,fontFamily:'system-ui, sans-serif', '--chat-link':'#2563eb'} as React.CSSProperties}><h1 style={{fontSize:20}}>机器人交互卡片</h1><p style={{fontSize:13,color:'#737984'}}>本地演示数据 · 不连接机器人</p><LiveCameraCard />{(['robot','joints','imu','navigation'] as const).map(view=><ObservationCard key={view} view={view}/>)}</main>)

