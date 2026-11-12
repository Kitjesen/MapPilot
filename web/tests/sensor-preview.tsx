import { useEffect, useState } from 'react'
import { createRoot } from 'react-dom/client'
import { Activity, ArrowUp, Moon, Plus, Settings2, Sun } from 'lucide-react'
import { SensorMonitorProvider } from '../src/components/SensorMonitor'
import { SettingsMenu } from '../src/components/SettingsMenu'
import { ComposerAdd } from '../src/components/agentConversation/ComposerAdd'
import { useSensorMonitor } from '../src/hooks/useSensorMonitor'
import { useTheme } from '../src/components/useTheme'
import type { SensorId } from '../src/services/sensors'
import type { SSEState } from '../src/types'
import '../src/App.css'
import './sensor-preview.css'

const state = { events: [], connected: false } as unknown as SSEState
const sets: Record<string, SensorId[]> = { motion: ['imu','joints','motors','voltages'], perception: ['camera','lidar','thermal','acoustic'], location: ['gnss','semantic'] }
const selected = new URLSearchParams(location.search).get('set') ?? ''
function Launcher() {
  const open = useSensorMonitor()
  const [draft, setDraft] = useState('')
  useEffect(() => { for (const id of sets[selected] ?? []) open?.(id) }, [open])
  return <>
    <div className="sensor-preview-hero"><Activity size={30} strokeWidth={1.5} /><h1>看见每一次变化。</h1><p>将需要的观测留在身边。<br />每个传感器，都有自己的位置。</p>
      <button className="preview-primary" onClick={event => open?.('catalog', event.currentTarget)}><Plus size={15} />打开组件</button>
      <nav><a href="?set=motion">运动数据 <span>IMU / 关节 / 电机 / 电压</span></a><a href="?set=perception">环境感知 <span>相机 / LiDAR / 热成像 / 声学</span></a><a href="?set=location">定位与语义 <span>GNSS / 语义真值</span></a></nav>
    </div>
    <div data-chat-slot="global" className="global-task-slot"><form className="sensor-preview-composer" onSubmit={event => event.preventDefault()}><input placeholder="打开你想查看的传感器…" aria-label="预览输入框" value={draft} onChange={event => setDraft(event.target.value)} /><div><ComposerAdd locale="zh" disabled={false} hasMap={false} onFiles={() => {}} onPrompt={setDraft} /><span>组件预览</span><button type="submit" disabled aria-label="预览不发送指令"><ArrowUp size={16} /></button></div></form><small className="sensor-preview-footnote">合成观测 · 相机为历史 MuJoCo 画面 · 不连接机器人</small></div>
  </>
}
export function Preview() {
  const [settings, setSettings] = useState(false)
  const [now, setNow] = useState(Date.now)
  const { theme, resolvedTheme, setTheme, toggle } = useTheme()
  useEffect(() => { const timer = setInterval(() => setNow(Date.now()), 1000); return () => clearInterval(timer) }, [])
  return <SensorMonitorProvider sseState={state} localNowS={now / 1000}>
    <main className="sensorPreview"><header><span><Activity size={18} /> LingTu <i>组件工作台</i></span><div><button aria-label="切换主题" onClick={toggle}>{resolvedTheme === 'dark' ? <Sun size={16} /> : <Moon size={16} />}</button><button aria-label="打开正式设置" onClick={() => setSettings(true)}><Settings2 size={16} /></button></div></header><Launcher />
      <SettingsMenu open={settings} onClose={() => setSettings(false)} theme={theme} resolvedTheme={resolvedTheme} onThemeChange={setTheme} locale="zh" onLocaleChange={() => {}} sseState={state} localNowS={now / 1000} showToast={() => {}} />
    </main>
  </SensorMonitorProvider>
}
createRoot(document.getElementById('root')!).render(<Preview />)
