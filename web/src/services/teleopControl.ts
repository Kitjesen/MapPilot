import type { NavigationStatusResponse, ProductName } from '../types/index.ts'

export function supportsWebTeleop(product: ProductName | null | undefined): boolean {
  return product === 'map' || product === 'teleop' || product === 'teleop_avoid' || product === 'nav'
}

/** A takeover requires confirmation before autonomy resumes, not between manual inputs. */
export function teleopInputBlocked(
  product: ProductName | null,
  navigation: NavigationStatusResponse | null,
): boolean {
  if (!navigation || navigation.control.authority === 'UNKNOWN') return true
  if (product !== 'nav') return navigation.control.resume_required

  const takeover = navigation.control.authority === 'OPERATOR'
    && navigation.control.reason === 'operator_takeover'
  if (navigation.motion.permission === 'ESTOPPED' || navigation.motion.permission === 'UNKNOWN') return true
  if (navigation.motion.permission === 'HELD'
    && !(takeover && navigation.motion.reason === 'operator_takeover')) return true
  return navigation.control.resume_required && !takeover
}

/** W3C standard layout: left stick translation, right stick yaw, L1/LB deadman. */
export function gamepadCommand(
  pad: Pick<Gamepad, 'connected' | 'mapping' | 'axes' | 'buttons'> | null,
  limits: { linearMps: number; yawRadS: number },
): { vxMps: number; vyMps: number; yawRps: number; deadman: boolean; neutral: boolean } | null {
  if (!pad?.connected || pad.mapping !== 'standard' || pad.axes.length < 3 || pad.buttons.length < 5) return null
  const axis = (value: number) => Math.abs(value) <= 0.15 ? 0 : Math.sign(value) * (Math.abs(value) - 0.15) / 0.85
  const forward = -axis(pad.axes[1])
  const lateral = -axis(pad.axes[0])
  const yaw = -axis(pad.axes[2])
  const length = Math.max(1, Math.hypot(forward, lateral))
  return {
    vxMps: forward / length * limits.linearMps,
    vyMps: lateral / length * limits.linearMps,
    yawRps: yaw * limits.yawRadS,
    deadman: pad.buttons[4].pressed,
    neutral: forward === 0 && lateral === 0 && yaw === 0,
  }
}
