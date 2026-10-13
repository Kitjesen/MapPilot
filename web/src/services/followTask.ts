import type { FollowTaskRef } from '../types'

interface FollowTaskProjection {
  follow_task_id?: string | null
  product_session_id?: string | null
  task_terminal?: boolean | null
}

export function selectDisplayedFollowTask(
  status: FollowTaskProjection | null | undefined,
  accepted: FollowTaskRef | null,
  currentProductSessionId: string | null | undefined,
): FollowTaskRef | null {
  if (!currentProductSessionId) return null

  const statusRef = status?.follow_task_id
    && status.product_session_id === currentProductSessionId
    ? {
        follow_task_id: status.follow_task_id,
        product_session_id: status.product_session_id,
      }
    : null
  const acceptedRef = accepted?.product_session_id === currentProductSessionId
    ? accepted
    : null

  if (
    acceptedRef
    && (
      !statusRef
      || statusRef.follow_task_id !== acceptedRef.follow_task_id
    )
  ) {
    return acceptedRef
  }
  if (statusRef && status?.task_terminal !== true) return statusRef
  return acceptedRef && status?.task_terminal !== true ? acceptedRef : null
}
