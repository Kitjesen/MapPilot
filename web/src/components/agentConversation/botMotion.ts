const blinkPhrases = [
  { duration: 8100, blinks: [1400, 5700] },
  { duration: 9400, blinks: [2300, 2650, 7400] },
  { duration: 10300, blinks: [3800, 8200] },
  { duration: 8700, blinks: [1900, 6300, 6650] },
] as const

// Original LingTu timing; each phrase owns its blinks, separate from head and gaze.
export function startBotBlink(root: SVGSVGElement, lids: SVGGElement) {
  const reduced = window.matchMedia('(prefers-reduced-motion: reduce)')
  let timers: ReturnType<typeof setTimeout>[] = []
  let blink: Animation | undefined
  let previous = -1
  let deck: number[] = []

  function stop() {
    timers.forEach(clearTimeout)
    timers = []
    blink?.cancel()
    blink = undefined
  }

  function play(first = false) {
    if (!deck.length) {
      deck = blinkPhrases.map((_, index) => index)
      for (let index = deck.length - 1; index > 0; index--) {
        const other = Math.floor(Math.random() * (index + 1))
        ;[deck[index], deck[other]] = [deck[other], deck[index]]
      }
      if (deck[0] === previous) [deck[0], deck[1]] = [deck[1], deck[0]]
    }
    const index = deck.shift()!
    previous = index
    const phrase = blinkPhrases[index]
    const offset = first ? Math.random() * phrase.duration : 0
    timers = phrase.blinks.filter(time => time >= offset).map(time => setTimeout(() => {
      blink?.cancel()
      blink = lids.animate([
        { transform: 'scaleY(1)', offset: 0 },
        { transform: 'scaleY(.06)', offset: .4 },
        { transform: 'scaleY(1)', offset: 1 },
      ], { duration: 240, easing: 'cubic-bezier(.4, 0, .6, 1)' })
    }, time - offset))
    timers.push(setTimeout(() => play(), phrase.duration - offset))
  }

  function sync() {
    stop()
    const paused = document.hidden || reduced.matches
    root.dataset.paused = String(paused)
    if (!paused) play(true)
  }

  document.addEventListener('visibilitychange', sync)
  reduced.addEventListener('change', sync)
  sync()
  return () => {
    stop()
    document.removeEventListener('visibilitychange', sync)
    reduced.removeEventListener('change', sync)
  }
}
