# 场景图：什么进“记忆”、有没有楼层、FSR-VLN 快慢怎么做

## 1. 什么东西会放进场景图（“记忆”）

**规则一句话**：所有被 YOLO-World 检测到、且通过「匹配/合并」或「新建」进入追踪表的物体都会进场景图，直到被 prune 掉。

### 1.1 进入条件

- **输入**：每一帧的 3D 检测列表（label, position, score, CLIP features）。
- **匹配**：对每个检测，在现有物体表里找「同 label + 距离 < merge_distance(0.5m)」的物体；若有 CLIP，再要求相似度 > clip_threshold(0.75)。  
  - 匹配到 → **更新**该物体（位置融合、信念更新、detection_count++）。  
  - 没匹配到 → **新建**一个物体，加入表。
- 因此：**只要被检测到且没和已有物体合并，就会进表**；没有“语义重要性”预筛，全靠后续的「数量上限 + 陈旧剔除」控制规模。

### 1.2 保留 / 剔除（谁留在“记忆”里）

- **数量上限**：`max_objects = 200`。超过时按 **(credibility, detection_count)** 降序排序，**只保留前 200 个**，其余从表中删除。
- **陈旧剔除**：超过 `stale_timeout = 300` 秒（5 分钟）未被看到的物体会被 **prune_stale()** 删除。

所以：  
- **放进机器**的 = 当前所有追踪到的物体（最多 200 个）。  
- **谁留下** = 可信度高、被看到次数多、且最近 5 分钟内被见过；否则会被删掉。

### 1.3 层次里到底有什么

场景图 JSON 里实际有的层级是：

- **objects**：物体列表（id, label, position, score, detection_count, region_id, belief）。
- **relations**：物体之间的空间关系（near, on, above, left_of 等）。
- **regions**：空间区域（DBSCAN 聚类，ε=3m），每个 region 有 name（如 area_with_door_desk）、center、object_ids。
- **rooms**：由 regions 加推理得到的“房间”节点（room_id, name, center, object_ids, group_ids）；name 由规则或可选 LLM 从区域内的物体推断（如 corridor, office）。
- **groups**：语义分组（如 office_workstation：桌+椅+显示器），挂在 room 下。
- **views**：关键视角节点（位置、时间戳、可见 object_ids、key_labels），用于多视角检索。

**结论**：  
- 没有单独的“物体→组→房间→**楼层**”里的 **楼层（Floor）** 节点。  
- 实际实现是 **物体 → 组 → 房间（Room）**，再加 **区域（Region）** 和 **View**；**楼层在当前代码里未实现**。  
- 论文里若写“四层：物体→组→房间→楼层”，需要改为“三层：物体→组→房间”，或后续在实现中补一层 Floor（例如单层楼用 floor_0）。

---

## 2. 我们和 FSR-VLN 的对比（以及 FSR-VLN 快慢怎么做）

### 2.1 FSR-VLN (2025) 的层次与快慢流程

- **HMSG 四层**：**Floor → Room → View → Object**（他们真的有 Floor）。  
  - View 节点存：CLIP embedding + **VLM 生成的图像描述**、相机位姿；每个 Object 关联一个“最佳视角”（能看到该物体且深度最近的 view）。  
  - 建图是**离线**的：先 SLAM + 实例级开放词汇建图，再建 HMSG。

- **快（Fast）**：  
  1. 用 **LLM** 把用户指令解析成结构化查询（floor / region / object）。  
  2. **CLIP 粗筛**：用查询文本的 CLIP 与图中各层特征做相似度：  
     - 先匹配 **Room**（若有指定房间）→ 候选房间；  
     - 再在候选房间内用 **View** 的 CLIP 匹配 → 候选 view；  
     - 再用 **Object** 的 CLIP 匹配 → 候选 object。  
  得到：候选 goal view、goal object（可能错）。

- **慢（Slow）**：  
  1. **VLM 验证**：用 **GPT-4o** 看“匹配到的 object 的 best view 图像”，判断指令里的目标是否真的出现在这张图里。若 VLM 说不在 → 认为该 object 不可靠。  
  2. **VLM 精筛**：若快匹配不可靠，用 LLM 对未匹配 view 的文本描述做推理，挑出最符合语义的一个 view（view-1），再用 VLM 比较“快匹配的 view”和“view-1”，选最终 goal image；再在该 view 对应的物体列表上重新跑 CLIP，更新 goal object。

所以：  
- **CLIP 粗筛** = 用 CLIP 在 Room → View → Object 上逐层做相似度，得到候选。  
- **VLM 精筛** = 用 VLM 看真实图像做“是否在图中”的验证和 refinement，减少 CLIP 误匹配。

### 2.2 我们 vs FSR-VLN（简要）

| 项目         | FSR-VLN (2025)              | 我们 (HSG-Nav)                    |
|--------------|-----------------------------|-----------------------------------|
| 建图         | 离线 HMSG                   | 在线增量（边走边建）               |
| 层次         | Floor → Room → View → Object | Object → Group → Room（无 Floor）  |
| View 层      | 有，CLIP+VLM 描述           | 有 View 节点，但未用 VLM 描述     |
| 快           | CLIP 逐层匹配               | 多源融合（标签+CLIP+检测分+空间） |
| 慢           | VLM 看图验证 + 精筛         | LLM 层次 CoT（文本场景图）        |
| 是否用 VLM 看图 | 是（GPT-4o 看图）           | 可选（我们主要用文本场景图）      |

---

## 3. 用网上视频 / 开放数据测 YOLO-World、CLIP、Slow Path

- **YOLO-World**：可用任意网络视频或公开数据集（如 COCO 视频、YouTube 室内片段）抽帧，跑检测；不依赖机器狗。  
- **CLIP**：同样用这些图像/帧，做文本–图像相似度或检索；可单独测。  
- **Slow Path（LLM）**：只需场景图 JSON + 指令；用 Moonshot/OpenAI 等 API 跑 `resolve()`，不依赖真机。  
  - 我们已提供 Moonshot 配置与 Slow Path 验证脚本，通过环境变量传入 API key，在本地即可验证 Slow Path 行为。

---

## 4. 参考文献

- FSR-VLN: *FSR-VLN: Fast and Slow Reasoning for Vision-Language Navigation with Hierarchical Multi-modal Scene Graph*, arXiv:2509.13733 (2025).  
- HMSG 为 Floor–Room–View–Object；快阶段 CLIP 粗筛，慢阶段 VLM 看图验证与精筛。
