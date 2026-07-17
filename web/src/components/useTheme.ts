import { useEffect, useState } from 'react'

export type Theme = 'system' | 'dark' | 'light'
export type ResolvedTheme = 'dark' | 'light'

function isTheme(value: string | null): value is Theme {
  return value === 'system' || value === 'dark' || value === 'light'
}

function resolveTheme(theme: Theme): ResolvedTheme {
  if (theme !== 'system') return theme
  if (typeof window === 'undefined') return 'dark'
  return window.matchMedia('(prefers-color-scheme: light)').matches ? 'light' : 'dark'
}

export function useTheme() {
  const [theme, setTheme] = useState<Theme>(() => {
    if (typeof window !== 'undefined') {
      const stored = localStorage.getItem('lingtu-theme')
      return isTheme(stored) ? stored : 'system'
    }
    return 'system'
  })
  const [resolvedTheme, setResolvedTheme] = useState<ResolvedTheme>(() => resolveTheme(theme))

  useEffect(() => {
    const apply = () => {
      const next = resolveTheme(theme)
      setResolvedTheme(next)
      document.documentElement.setAttribute('data-theme', next)
      document.documentElement.setAttribute('data-theme-choice', theme)
    }
    apply()
    localStorage.setItem('lingtu-theme', theme)

    if (theme !== 'system' || typeof window === 'undefined') return
    const media = window.matchMedia('(prefers-color-scheme: light)')
    media.addEventListener('change', apply)
    return () => media.removeEventListener('change', apply)
  }, [theme])

  const toggle = () => setTheme(resolveTheme(theme) === 'dark' ? 'light' : 'dark')
  return { theme, resolvedTheme, setTheme, toggle }
}
