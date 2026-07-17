export type Locale = 'en' | 'zh'

const STORAGE_KEY = 'lingtu.locale'
const DEFAULT_LOCALE: Locale = 'en'

export function normalizeLocale(value: unknown): Locale {
  return value === 'zh' || value === 'en' ? value : DEFAULT_LOCALE
}

export function readStoredLocale(): Locale {
  if (typeof window === 'undefined') return DEFAULT_LOCALE
  try {
    return normalizeLocale(window.localStorage.getItem(STORAGE_KEY))
  } catch {
    return DEFAULT_LOCALE
  }
}

export function writeStoredLocale(locale: Locale): void {
  if (typeof window === 'undefined') return
  try {
    window.localStorage.setItem(STORAGE_KEY, locale)
  } catch {
    // Storage can be unavailable in restricted browser contexts.
  }
}

export function text(locale: Locale, en: string, zh: string): string {
  return locale === 'zh' ? zh : en
}
