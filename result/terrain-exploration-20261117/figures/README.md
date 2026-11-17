# 连续地形测试图包

新增 36 个场景、86 组请求。72 组通过当前全局规划与几何复核，14 组没有路线。
这是合成地形 OctoMap 上的静态规划测试，不是机器人动力学或真实运动录像。

- `index.html`：全部场景的离线图册。
- `png/`：600 dpi 图片；`pdf/`、`svg/`：矢量图。
- `terrain_overview`：波浪、深起伏、双向起伏、坡顶／坡脚、坡上 S 弯总览。
- `terrain_boundaries`：横坡、窄脊、缺测、低顶棚和无连接楼层。
- `model_boundaries`：解释为什么几何通过仍不足以证明姿态和足端支撑可行。
- 其余 36 张场景图展示该场景的全部请求，并附路线 XZ 投影。
- `metrics.csv`、`analysis.json`：逐请求结果和沿路线的采样诊断。
- `scenes.json`、`results.json`、`options.json`：冻结地形、路线、原始复核结果和配置。
- `export.py`、`export_boundaries.py`：使用本目录冻结数据重新绘制。

三维视图采用等米制尺度；右侧 XZ 曲线图为了阅读独立缩放纵横坐标，标题中已注明。
三维表面是高度采样展示，规划实际使用 5 cm OctoMap。颜色用于表达单张图内的高度，
不能跨图直接比较颜色。未找到路线时不绘制虚构连接线。

横坡诊断以设计高度场为真值，沿已返回路线按 5 cm 采样计算；这些诊断没有送入
规划器作为额外通行条件。身体包络投影下地面高差不等于实际各足落点高差。
无路线时没有可采样的路线，对应诊断留空，不填成零。

35／45° 横坡、20 cm 窄脊的通过结果暴露了当前姿态／足支撑覆盖不足。
不得把这些图写成“机器人已稳定通过对应横坡或窄脊”。本轮修复的是保留输入
已有站立高度的搜索问题，不包含自动抬升身体或足端控制。

完整调查见 [研究记录](../../research/terrain-exploration-20261117.md)。
原生地图和复测入口位于项目 `build/terrain-exploration-20261117/`。

图像重绘使用现有 Python 环境中的 Matplotlib 和 NumPy：

```powershell
uv run python docs/media/terrain-exploration-20261117/export.py
uv run python docs/media/terrain-exploration-20261117/export_boundaries.py
```
