export type MapResetEpoch = string

/** Preserve reset epochs as opaque decimal identities without IEEE-754 rounding. */
export function mapResetEpoch(value: unknown): MapResetEpoch | null {
  if (typeof value === 'string') {
    return /^(?:0|[1-9][0-9]*)$/.test(value) ? value : null
  }
  if (typeof value === 'number' && Number.isSafeInteger(value) && value >= 0) {
    return String(value)
  }
  return null
}
