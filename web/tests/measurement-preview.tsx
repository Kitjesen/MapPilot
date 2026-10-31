import { createRoot } from 'react-dom/client'
import MeasurementInspector from '../src/components/MeasurementInspector'
import '../src/App.css'

createRoot(document.getElementById('root')!).render(<main style={{ maxWidth: 1100, margin: '32px auto', padding: 24 }}>
  <h1>多模态测量检查器</h1>
  <p>隔离界面验证：真实 MuJoCo 测量夹具，不是工厂 Product，不启动导航或机器人运动。</p>
  <p>红色热源：400 K；背景墙：300 K；声源：200 Hz 合成标定信号。控制走原生 DDS，图像数据在本地解码传递。</p>
  <MeasurementInspector locale="zh" />
</main>)
