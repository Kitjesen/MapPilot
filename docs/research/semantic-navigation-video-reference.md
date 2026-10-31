# 语义导航视频参考与验收

**更新：** 2026-10-05

## 结论

`artifacts/water-bottle-search-demo-20261005/` 中的 MP4/GIF 是接口诊断素材，
不是论文复现，也不是机器狗真实执行记录。它没有引用任何论文视频，不能作为
LingTu 的语义导航结果展示。

## 主要参考

### HOV-SG（RSS 2024）

- 论文：[Hierarchical Open-Vocabulary 3D Scene Graphs for Language-Grounded Robot Navigation](https://arxiv.org/abs/2403.17846)
- 官方页面：[hovsg.github.io](https://hovsg.github.io/)
- 官方实机视频：[YouTube](https://www.youtube.com/watch?v=GC-Q0ekO9qg)

这是 LingTu 当前最适合的主参考。论文链路是：

```text
RGB-D + 里程计/激光位姿
  -> 3D segment map + VLM 特征
  -> floor / room / object 层级场景图
  -> 语言查询分解
  -> 导航图与子目标
  -> Spot 机器人运动
```

论文的真实实验使用 Boston Dynamics Spot、Azure Kinect RGB-D 和 3D LiDAR，
在两层办公楼中测试物体、房间和楼层查询；报告的 object goal navigation 成功率
为 56.1%（41 次试验中的 23 次）。这个结果属于论文作者的实验，不是 LingTu 的结果。

### OneMap（ICRA 2025）

- 论文主页：[One Map to Find Them All](https://www.finnbusch.com/OneMap/)
- 论文：[arXiv](https://arxiv.org/abs/2409.11764)
- 官方 Spot 视频：[YouTube](https://www.youtube.com/watch?v=e3gqWM9QkTg)

OneMap 更贴近“帮我找一瓶水”这种开放词汇物体搜索：它在 Spot 上维护可复用的
开放词汇特征地图，并利用语义不确定性连续搜索多个物体。它适合作为 HOV-SG
目标检索和持续搜索部分的第二个参考。

## 下一段 LingTu 视频必须包含

1. 数据集或真实传感器来源、相机/激光位姿和标定来源；
2. RGB/深度或点云中的目标证据，而不是手工放置的 `synthetic_contract_fixture`；
3. floor → room → object（或 object search）查询结果和置信度；
4. 目标几何引用、原生 planner 输入和 planner 接受/拒绝原因；
5. 俯视图路径与机器人视角/仿真轨迹同步；
6. 明确标注 `dataset_simulation`、`native_no_motion` 或 `real_robot_motion`；
7. 如果目标不存在、定位失败或 planner 拒绝，视频必须展示失败原因，不得用虚线代替实走。

在满足这些条件前，LingTu 只可以声称完成了某个数据集上的离线验证，不能声称
复现 HOV-SG/OneMap 的论文结果，也不能声称机器狗已经通过文本找到水瓶。
