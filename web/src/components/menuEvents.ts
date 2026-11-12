import { useEffect, type RefObject } from 'react'

export function useMenuDismiss(root: RefObject<HTMLElement | null>, enabled = true): void {
  useEffect(() => {
    const element = root.current
    if (!enabled || !element) return
    const menus = () => element.querySelectorAll<HTMLDetailsElement>('details[name="lingtu-menu"], details[data-hover-menu]')
    const pending = new Map<HTMLDetailsElement, { timer?: ReturnType<typeof setTimeout>, openedByHover: boolean }>()
    const cancel = (menu: HTMLDetailsElement) => {
      const state = pending.get(menu)
      if (state?.timer !== undefined) clearTimeout(state.timer)
      return state
    }
    const hover = (event: globalThis.PointerEvent) => {
      if (event.pointerType !== 'mouse') return
      const entering = event.type === 'pointerover'
      for (const menu of menus()) {
        if (!menu.hasAttribute('data-hover-menu') || !menu.contains(event.target as Node)
          || menu.contains(event.relatedTarget as Node | null)) continue
        const state = cancel(menu) ?? { openedByHover: false }
        pending.set(menu, state)
        state.timer = setTimeout(() => {
          state.timer = undefined
          if (!menu.hasAttribute('data-hover-menu')) return
          if (entering) {
            if (!menu.open) {
              menu.open = true
              state.openedByHover = true
            }
          } else {
            menu.open = false
            state.openedByHover = false
          }
        }, entering ? 220 : 180)
      }
    }
    const click = (event: MouseEvent) => {
      for (const menu of menus()) {
        if (!menu.querySelector('summary')?.contains(event.target as Node)) continue
        const state = cancel(menu)
        // A mouse click after hover must not immediately undo the hover opening.
        if (state?.openedByHover && menu.open && event.detail > 0) event.preventDefault()
        if (state) state.openedByHover = false
      }
    }
    const outside = (event: globalThis.PointerEvent) => {
      for (const menu of menus()) {
        if (!menu.contains(event.target as Node)) {
          cancel(menu)
          menu.open = false
        } else if (event.pointerType !== 'mouse') {
          const state = cancel(menu)
          if (state) state.openedByHover = false
        }
      }
    }
    const escape = (event: KeyboardEvent) => {
      if (event.key !== 'Escape' || event.defaultPrevented) return
      let summary: HTMLElement | null = null
      for (const menu of menus()) {
        cancel(menu)
        if (menu.open) {
          menu.open = false
          summary ??= menu.querySelector('summary')
        }
      }
      summary?.focus()
    }
    element.addEventListener('pointerover', hover)
    element.addEventListener('pointerout', hover)
    element.addEventListener('click', click)
    document.addEventListener('pointerdown', outside)
    document.addEventListener('keydown', escape)
    return () => {
      for (const menu of pending.keys()) cancel(menu)
      element.removeEventListener('pointerover', hover)
      element.removeEventListener('pointerout', hover)
      element.removeEventListener('click', click)
      document.removeEventListener('pointerdown', outside)
      document.removeEventListener('keydown', escape)
    }
  }, [root, enabled])
}
