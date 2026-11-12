import type { Locale } from '../i18n'
import type {
  NavigationControlAuthority,
  NavigationGoalAdmission,
  NavigationMotionObservation,
  NavigationMotionPermission,
  NavigationStopConfirmation,
  NavigationTaskState,
  NavigationStatusResponse,
  SSEState,
} from '../types/index.ts'
import { estimateSceneTime, freshSource } from './sceneTelemetry.ts'

/** Older navigation status is presented as unknown, never as the last good state. */
export const NAVIGATION_STATUS_MAX_AGE_S = 7

/** HTTP snapshots and SSE share Gateway timestamps; delayed replies must not rewind them. */
export function latestNavigationStatus(current: NavigationStatusResponse | null, incoming: NavigationStatusResponse | null | undefined): NavigationStatusResponse | null {
  if (!incoming || !Number.isFinite(incoming.ts)) return current
  return current && current.ts > incoming.ts ? current : incoming
}

/** Return navigation status only while the stream is connected and the status is fresh. */
export function liveNavigationStatus(
  sseState: Pick<SSEState, 'connected' | 'navigationStatus' | 'stateSnapshot' | 'stateSnapshotReceivedAt'>,
  localNowS: number,
): NavigationStatusResponse | null {
  const status = sseState.navigationStatus
  const nowS = estimateSceneTime(localNowS, sseState.stateSnapshot?.ts, sseState.stateSnapshotReceivedAt)
  return sseState.connected && freshSource(status?.ts, nowS, NAVIGATION_STATUS_MAX_AGE_S) ? status : null
}

interface LabeledState<T extends string> {
  state: T
  label: string
}

export interface NavigationStatusPresentation {
  task: {
    state: NavigationTaskState
    label: string
  }
  goalAdmission: LabeledState<NavigationGoalAdmission> & {
    reason: string
    message: string
  }
  control: LabeledState<NavigationControlAuthority> & {
    resumeRequired: boolean
  }
  motion: {
    permission: LabeledState<NavigationMotionPermission>
    observation: LabeledState<NavigationMotionObservation>
    stopConfirmation: LabeledState<NavigationStopConfirmation>
    reason: string
    message: string
  }
}

const TASK_LABELS: Record<NavigationTaskState, { en: string; zh: string }> = {
  IDLE: { en: 'Idle', zh: '空闲' },
  PLANNING: { en: 'Planning', zh: '规划中' },
  EXECUTING: { en: 'Executing', zh: '执行中' },
  RECOVERING: { en: 'Recovering', zh: '恢复中' },
  PAUSED: { en: 'Paused', zh: '已暂停' },
  SUCCESS: { en: 'Reached', zh: '已到达' },
  FAILED: { en: 'Failed', zh: '失败' },
  CANCELLED: { en: 'Cancelled', zh: '已取消' },
  UNKNOWN: { en: 'Unknown', zh: '未知' },
}

const MOTION_PERMISSION_LABELS: Partial<Record<NavigationMotionPermission, { en: string; zh: string }>> = {
  CLEAR: { en: 'Motion clear', zh: '允许运动' },
  HELD: { en: 'Motion held', zh: '运动保持' },
  ESTOPPED: { en: 'Emergency stop held', zh: '急停保持' },
  UNKNOWN: { en: 'Motion permission unknown', zh: '运动权限未知' },
}

const MOTION_OBSERVATION_LABELS: Partial<Record<NavigationMotionObservation, { en: string; zh: string }>> = {
  MOVING: { en: 'Moving', zh: '运动中' },
  QUIET: { en: 'Observed quiet', zh: '静止观测' },
  UNKNOWN: { en: 'Motion state unknown', zh: '运动状态未知' },
}

const STOP_CONFIRMATION_LABELS: Partial<Record<NavigationStopConfirmation, { en: string; zh: string }>> = {
  NOT_REQUESTED: { en: 'Stop not requested', zh: '未请求停止确认' },
  PENDING: { en: 'Awaiting stop confirmation', zh: '等待停止确认' },
  CONFIRMED: { en: 'Stop command confirmed', zh: '停止命令已确认' },
  FAILED: { en: 'Stop confirmation failed', zh: '停止确认失败' },
  UNKNOWN: { en: 'Stop confirmation unknown', zh: '停止确认状态未知' },
}

const GOAL_ADMISSION_LABELS: Partial<Record<NavigationGoalAdmission, { en: string; zh: string }>> = {
  ACCEPTING: { en: 'Accepting goals', zh: '可接收目标' },
  BLOCKED: { en: 'Goal blocked', zh: '目标受阻' },
  UNKNOWN: { en: 'Goal admission unknown', zh: '目标准入未知' },
}

const CONTROL_AUTHORITY_LABELS: Partial<Record<NavigationControlAuthority, { en: string; zh: string }>> = {
  AUTONOMY: { en: 'Autonomy control', zh: '自主控制' },
  OPERATOR: { en: 'Operator control', zh: '操作者控制' },
  NONE: { en: 'No controller', zh: '无控制者' },
  UNKNOWN: { en: 'Control authority unknown', zh: '控制权未知' },
}

function labelState<T extends string>(
  state: T,
  labels: Partial<Record<T, { en: string; zh: string }>>,
  locale: Locale,
): LabeledState<T> {
  const label = labels[state]
  return { state, label: label ? label[locale] : state }
}

function normalizeTaskState(value: unknown): NavigationTaskState {
  if (typeof value !== 'string' || !value.trim()) return 'UNKNOWN'
  const candidate = value.trim().toUpperCase()
  return candidate in TASK_LABELS ? candidate as NavigationTaskState : 'UNKNOWN'
}

function localizationRecoveryMessage(
  reason: string,
  stopConfirmation: NavigationStopConfirmation,
  locale: Locale,
): string {
  if (!reason.startsWith('localization_')) return ''
  if (stopConfirmation === 'CONFIRMED') {
    return locale === 'zh'
      ? '定位恢复中，停止命令已确认'
      : 'Localization recovery in progress; stop command confirmed'
  }
  if (stopConfirmation === 'PENDING') {
    return locale === 'zh'
      ? '定位恢复中，正在等待停止确认'
      : 'Localization recovery in progress; awaiting stop confirmation'
  }
  if (stopConfirmation === 'FAILED') {
    return locale === 'zh'
      ? '定位恢复中，停止确认失败'
      : 'Localization recovery in progress; stop confirmation failed'
  }
  return locale === 'zh'
    ? '定位恢复中，运动指令已保持'
    : 'Localization recovery in progress; motion commands are held'
}

export function navigationTaskLabel(
  state: NavigationTaskState,
  locale: Locale,
  reason = '',
): string {
  if (state === 'PLANNING' && reason === 'planning_waiting') {
    return operatorReasonMessage(reason, locale)
  }
  if (['EXECUTING', 'RECOVERING'].includes(state) && reason === 'replan_waiting') {
    return operatorReasonMessage(reason, locale)
  }
  return TASK_LABELS[state][locale]
}

export function navigationTaskReasonMessage(reason: string): string {
  // Localization gaps are a motion hold, not a terminal timeout.  Keep the
  // native reason available to the generic operator copy instead of reviving
  // the old timeout message for stale status records.
  return reason
}

/**
 * Convert native reason codes into the short message an operator needs.
 * Known reasons explain the affected step; unknown values stay visible for diagnosis.
 */
export function operatorReasonMessage(reason: string, locale: Locale): string {
  const value = reason.trim()
  if (!value) return ''

  const messages: Array<[string, string, string]> = [
    ['navigation_session_inactive', '机器人导航服务未启动', 'Robot navigation service is not active'],
    ['preview_result_invalid', '预览未返回与所选位置对应的有效终点，请重试。', 'The preview did not return a valid endpoint for the selected target. Try again.'],
    ['goal_outside_static_map', '目标超出当前地图范围，请在地图内重新选点。', 'The goal is outside the current map. Select a point inside it.'],
    ['goal_snap_exhausted', '目标附近没有满足地面支撑与通行条件的位置，请重新选点。', 'No nearby goal meets ground support and passage requirements. Select another point.'],
    ['start_occupied', '机器人当前位置与地图障碍相交，请检查定位和周围环境。', 'The robot position intersects a mapped obstacle. Check localization and the surroundings.'],
    ['start_unsupported', '机器人当前位置缺少有效地面支撑，请检查定位和地图。', 'The robot position lacks valid ground support. Check localization and the map.'],
    ['start_temporary_obstacle', '导航起点被当前障碍占用，请先检查周围环境。', 'The navigation start is occupied by a current obstacle. Check the surroundings.'],
    ['start_outside_static_map', '机器人当前位置超出地图范围，请检查定位或加载的地图。', 'The robot position is outside the map. Check localization or the loaded map.'],
    ['start_snap_exhausted', '无法确定可执行的导航起点，请检查定位与地面支撑。', 'No executable start was found. Check localization and ground support.'],
    ['goal_not_reached', '规划终点未满足目标容差，请调整目标位置。', 'The planned endpoint does not meet the goal tolerance. Adjust the target.'],
    ['search_iteration_limit', '路径搜索达到计算上限，尚未找到可用路线。', 'The search budget was exhausted before a usable route was found.'],
    ['preview_timeout', '路线预览超时，请稍后重试。', 'The route preview timed out. Try again.'],
    ['preview_cancelled', '路线预览已取消。', 'The route preview was cancelled.'],
    ['planner_error', '规划服务发生错误，请查看详细原因。', 'The planner reported an error. See the details.'],
    ['planner_busy', '规划器正在处理其他请求，请稍后重试。', 'The planner is busy. Try again shortly.'],
    ['navigation_busy', '导航正在处理其他任务，请先查看当前任务。', 'Navigation is busy. Check the current task first.'],
    ['odometry_not_ready', '机器人定位尚未就绪，请等待有效位置。', 'Robot localization is not ready. Wait for a valid pose.'],
    ['map_odom_tf_not_ready', '地图与定位坐标尚未对齐，请等待定位就绪。', 'Map and localization frames are not aligned yet.'],
    ['active_octomap_not_configured', '当前导航地图尚未加载，请先选择可用于导航的地图。', 'No navigation map is configured. Load a navigation-ready map.'],
    ['planner_map_identity_missing', '无法核实规划使用的地图，请重新加载导航地图。', 'The planner map identity is missing. Reload the navigation map.'],
    ['active_map_changed_during_planning', '规划期间地图已改变，请重新预览路线。', 'The map changed during planning. Preview the route again.'],
    ['frame_epoch_changed', '规划期间坐标基准已改变，请重新预览路线。', 'The coordinate frame changed during planning. Preview the route again.'],
    ['operator_pause', '任务已由操作者暂停。', 'The task was paused by the operator.'],
    ['support_disconnected', '起点与目标之间的地面支撑尚未连通。', 'The observed ground support does not yet connect the start and goal.'],
    ['start_body_occupied', '起点机身与地图障碍物相交。', 'The robot body intersects a mapped obstacle at the start.'],
    ['goal_body_occupied', '目标位置的机身与地图障碍物相交。', 'The robot body intersects a mapped obstacle at the goal.'],
    ['collision_stale', '等待实时雷达地图更新。', 'Waiting for the live LiDAR map to update.'],
    ['local_collision_missing', '等待实时雷达地图。', 'Waiting for the live LiDAR map.'],
    ['odom_missing', '等待有效位置数据。', 'Waiting for valid position data.'],
    ['odom_stale', '等待位置数据更新。', 'Waiting for position data to update.'],
    ['tf_missing', '等待地图与定位坐标对齐。', 'Waiting for the map and localization frames to align.'],
    ['tf_stale', '等待坐标变换更新。', 'Waiting for the coordinate transform to update.'],
    ['driver_control_missing', '等待驱动连接。', 'Waiting for the driver connection.'],
    ['driver_control_stale', '等待驱动连接恢复。', 'Waiting for the driver connection to recover.'],
    ['driver_control_not_ready', '等待驱动就绪。', 'Waiting for the driver to become ready.'],
    ['driver_control_rejected', '驱动尚未允许接收运动指令。', 'The driver is not ready to accept motion commands.'],
    ['takeover_active', '人工接管尚未退出，请恢复自主导航。', 'Manual takeover is still active; resume autonomous navigation.'],
    // Recorded status retains the earlier takeover reason.
    ['operator_takeover_resume_required', '人工接管尚未退出，请恢复自主导航。', 'Manual takeover is still active; resume autonomous navigation.'],
    ['loop_hold', '控制循环暂停运动，等待恢复。', 'Motion is held while the control loop recovers.'],
    ['control_hold', '运动暂时保持，等待控制恢复。', 'Motion is held while control recovers.'],
    ['brake_blocked', '制动空间受阻，已请求停车。', 'Braking space is blocked. A stop was requested.'],
    ['cmd_blocked', '速度指令没有可用制动空间，已请求停车。', 'The command has no braking space. A stop was requested.'],
    ['cmd_limited', '速度指令已按制动空间降低。', 'Command speed was reduced to fit braking space.'],
    ['estop_latched', '急停仍处于锁定状态。', 'The emergency stop remains latched.'],
    ['zero_barrier_pending', '等待前一个控制来源结束运动。', 'Waiting for the previous control source to finish motion.'],
    ['authority_busy', '另一个操作端正在控制机器人。', 'Another operator is controlling the robot.'],
    ['authority_lease_expired', '遥控连接已过期，请重新接管。', 'The manual control connection expired; take control again.'],
    ['sample_stamp_stale', '遥控输入已过期，等待新输入。', 'The manual input expired; waiting for fresh input.'],
    ['no_active_authority', '尚未取得人工控制权。', 'Manual control has not been claimed.'],
    ['stair_approach', '正在接近楼梯入口。', 'Approaching the stair entrance.'],
    ['stair_aligning', '正在对齐楼梯入口朝向。', 'Aligning with the stair entrance.'],
    ['stair_switching', '正在切换楼梯模式。', 'Switching to the stair motion profile.'],
    ['stair_ascending', '正在上楼。', 'Ascending the stairs.'],
    ['stair_descending', '正在下楼。', 'Descending the stairs.'],
    ['stair_settling', '等待机器人停稳后恢复行走模式。', 'Waiting for the robot to settle before restoring the walking profile.'],
    ['stair_restoring', '正在恢复进入楼梯前的行走模式。', 'Restoring the motion profile used before the stairs.'],
    ['stair_profile_wait', '等待楼梯模式切换确认。', 'Waiting for confirmation of the stair motion profile.'],
    ['stair_profile_unsupported', '当前机器人不支持所需楼梯模式。', 'The robot does not support the required stair motion profile.'],
    ['stair_landing_space', '楼梯入口平台空间不足，无法容纳整机。', 'The stair entrance landing has insufficient space for the whole robot.'],
    ['stair_entry_required', '需要先到楼梯入口平台。', 'The robot must first reach the stair entrance landing.'],
    ['stair_hold_on_flight', '仍处于楼梯段，保留当前楼梯模式。', 'The robot is still on the stair flight; the current stair motion profile is retained.'],
    ['stair_route_wait', '等待满足楼梯约束的安全路线。', 'Waiting for a safe route that meets the stair constraints.'],
    ['stair_exit_target', '等待出口平台满足整机退出条件。', 'Waiting for the exit landing to allow the whole robot to leave the stairs.'],
    ['profile_transition', '正在切换行走模式，运动指令暂时保持。', 'The motion profile is changing; motion commands are temporarily held.'],
    ['driver_control_profile_transition', '正在切换行走模式，运动指令暂时保持。', 'The motion profile is changing; motion commands are temporarily held.'],
    ['input_gate_driver_control_profile_transition', '正在切换行走模式，运动指令暂时保持。', 'The motion profile is changing; motion commands are temporarily held.'],
    ['planning_waiting', '等待通路，自动重试规划', 'Waiting for a clear route; planning will retry automatically'],
    ['replan_waiting', '等待通路，自动继续', 'Waiting for a clear route; navigation will continue automatically'],
    ['replan_budget_consumed', '自动重试机会已用完。', 'Automatic retry attempts have been exhausted.'],
    ['replan_failed_budget_consumed', '重规划失败，自动重试机会已用完。', 'Replanning failed; automatic retry attempts have been exhausted.'],
    ['local_no_path', '暂未生成局部路线。', 'No local route is available yet.'],
    ['local_blocked', '局部路线受阻。', 'The local route is blocked.'],
    ['local_near_field_stop', '检测到近场障碍，已请求停车。', 'A near-field obstacle was detected; a stop was requested.'],
    ['execution_stalled', '运动进展不足，请查看当前任务状态。', 'Motion progress is insufficient. Check the current task state.'],
    ['local_recovery_exhausted', '局部恢复未找到可行运动，等待重新规划。', 'Local recovery found no feasible motion; waiting for replanning.'],
    ['empty_path', '暂未生成可执行路线。', 'No executable route is available yet.'],
    ['localization_not_tracking', '定位尚未进入跟踪状态，运动指令已保持。', 'Localization is not tracking; motion commands are held.'],
    ['collision_at_trajectory_start', '轨迹起点存在碰撞，运动指令未获放行。', 'A collision exists at the trajectory start; motion was not authorized.'],
    ['visual_servo_target_lost', '目标人员已丢失，已请求取消跟随导航。', 'The target person was lost; cancellation of follow navigation was requested.'],
    ['visual_servo_perception_stale', '相机感知已过期，已请求取消视觉导航。', 'Camera perception is stale; cancellation of visual navigation was requested.'],
  ]
  const nativeReason = value.startsWith('input_gate_') ? value.slice('input_gate_'.length) : value
  if (nativeReason.startsWith('stop_confirmation_timeout_')) {
    return locale === 'zh' ? '停车确认超时，运动指令已保持，请检查机器人状态。' : 'Stop confirmation timed out; motion commands are held. Check the robot state.'
  }
  if (nativeReason.includes('stop_timeout:') || nativeReason.includes('rpc_timeout:')) {
    return locale === 'zh' ? '机器人运动接口响应超时，运动指令已保持。' : 'The robot motion API timed out; motion commands are held.'
  }
  const match = messages.find(([code]) => nativeReason === code || nativeReason.startsWith(`${code}:`))
  if (match) return locale === 'zh' ? match[1] : match[2]

  if (/^[a-z][a-z0-9_:-]*$/i.test(value)) {
    return locale === 'zh' ? `导航状态：${value}` : `Navigation status: ${value}`
  }
  return value
}

export function presentNavigationStatus(
  status: NavigationStatusResponse | null | undefined,
  locale: Locale,
): NavigationStatusPresentation {
  const state = normalizeTaskState(status?.task?.state)
  const motion = status?.motion
  const goalAdmission = status?.goal_admission
  const control = status?.control
  const motionReason = motion?.reason ?? ''
  const stopConfirmation = motion?.stop_confirmation ?? 'UNKNOWN'
  const localizationRecovery = localizationRecoveryMessage(motionReason, stopConfirmation, locale)
  const goalAdmissionReason = goalAdmission?.reason ?? ''

  return {
    task: { state, label: navigationTaskLabel(state, locale, status?.task?.reason) },
    goalAdmission: {
      ...labelState(goalAdmission?.state ?? 'UNKNOWN', GOAL_ADMISSION_LABELS, locale),
      reason: goalAdmissionReason,
      message: goalAdmissionReason.startsWith('localization_')
        ? locale === 'zh'
          ? '定位恢复中，新目标会排队等待输入恢复'
          : 'Localization recovery in progress; new goals stay queued until inputs recover'
        : '',
    },
    control: {
      ...labelState(control?.authority ?? 'UNKNOWN', CONTROL_AUTHORITY_LABELS, locale),
      resumeRequired: control?.resume_required === true,
    },
    motion: {
      permission: labelState(motion?.permission ?? 'UNKNOWN', MOTION_PERMISSION_LABELS, locale),
      observation: labelState(motion?.observation ?? 'UNKNOWN', MOTION_OBSERVATION_LABELS, locale),
      stopConfirmation: labelState(stopConfirmation, STOP_CONFIRMATION_LABELS, locale),
      reason: motionReason,
      message: localizationRecovery,
    },
  }
}
