// Local frames for the isolated preview. This replaces WebSocket; it never forwards.
export const previewCamera = { active: 0, opened: 0, closed: 0, paused: false }
export function installPreviewCamera() {
  class Socket {
    binaryType = 'blob'
    readyState = 0
    onopen: ((event: Event) => void) | null = null
    onclose: ((event: Event) => void) | null = null
    onerror: ((event: Event) => void) | null = null
    onmessage: ((event: MessageEvent) => void) | null = null
    timer: ReturnType<typeof setInterval> | undefined
    constructor(url: string | URL) {
      if (new URL(String(url)).pathname !== '/ws/camera') throw new Error('Preview blocks external WebSockets')
      previewCamera.active++; previewCamera.opened++
      const canvas = document.createElement('canvas'); canvas.width = 640; canvas.height = 360
      const context = canvas.getContext('2d')!
      const frame = () => {
        if (previewCamera.paused || this.readyState !== 1) return
        context.fillStyle = '#dde5ec'; context.fillRect(0, 0, 640, 360)
        context.fillStyle = '#c6d0da'; context.beginPath(); context.moveTo(0, 360); context.lineTo(240, 170); context.lineTo(400, 170); context.lineTo(640, 360); context.fill()
        context.strokeStyle = '#9fadb9'; context.lineWidth = 2
        for (let x = 0; x <= 640; x += 80) { context.beginPath(); context.moveTo(320, 170); context.lineTo(x, 360); context.stroke() }
        context.fillStyle = '#b6c3cf'; context.fillRect(64, 90, 140, 150); context.fillRect(444, 108, 120, 130)
        context.fillStyle = '#8e9eac'; context.fillRect(270, 110, 100, 75)
        context.fillStyle = '#273848'; context.fillRect(12, 12, 235, 29)
        context.font = '14px sans-serif'; context.fillStyle = 'white'; context.fillText('本地演示画面 · 非机器人相机', 22, 32)
        context.fillStyle = '#273848'; context.font = '16px monospace'; context.fillText(new Date().toLocaleTimeString(), 516, 338)
        canvas.toBlob(blob => { if (blob && this.readyState === 1 && !previewCamera.paused) this.onmessage?.(new MessageEvent('message', { data: blob })) }, 'image/jpeg', .8)
      }
      queueMicrotask(() => { if (this.readyState !== 0) return; this.readyState = 1; this.onopen?.(new Event('open')); frame(); this.timer = setInterval(frame, 120) })
    }
    close() {
      if (this.readyState === 3) return
      clearInterval(this.timer); this.readyState = 3; previewCamera.active--; previewCamera.closed++
      this.onclose?.(new Event('close'))
    }
  }
  window.WebSocket = Socket as unknown as typeof WebSocket
}
