export interface AuthoritativeTruthObservation {
  authoritativeStateSeen: boolean
  lastTruthAt: number | null
  truthError: string | null
}

export type AuthoritativeTruthOutcome =
  | { ok: true }
  | { ok: false; error: unknown }

function errorMessage(error: unknown): string {
  if (error instanceof Error) return error.message
  const message = String(error ?? '').trim()
  return message || 'authoritative_state_refresh_failed'
}

export function observeAuthoritativeTruth(
  previous: AuthoritativeTruthObservation,
  outcome: AuthoritativeTruthOutcome,
  observedAt: number = Date.now(),
): AuthoritativeTruthObservation {
  if (!outcome.ok) {
    return {
      ...previous,
      truthError: errorMessage(outcome.error),
    }
  }
  return {
    authoritativeStateSeen: true,
    lastTruthAt: observedAt,
    truthError: null,
  }
}
