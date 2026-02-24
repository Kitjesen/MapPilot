# 3D-NAV API重构方案

**日期**: 2026-02-17
**目标**: 提取统一的API接口层，方便代码维护和对外集成

---

## 1. 执行摘要

### 1.1 为什么需要API重构？

**当前问题**:
- ❌ 各模块直接耦合，难以独立测试
- ❌ 没有统一的接口规范
- ❌ 对外集成困难（需要了解内部实现）
- ❌ 代码修改影响范围大
- ❌ 难以替换底层实现

**重构目标**:
- ✅ 统一的API接口层
- ✅ 模块解耦，独立可测
- ✅ 清晰的对外接口
- ✅ 易于维护和扩展
- ✅ 支持多种实现方式

### 1.2 重构范围

```
当前架构:
ROS2 Node → 直接调用内部类 → 紧耦合

重构后:
ROS2 Node → API接口层 → 内部实现 → 松耦合
```

**涉及模块**:
1. Semantic Perception API
2. Semantic Planner API
3. Scene Graph API
4. LLM Client API
5. Action Executor API

---

## 2. 当前架构分析

### 2.1 Semantic Perception 当前结构

```python
# 当前: perception_node.py 直接使用内部类
class SemanticPerceptionNode(Node):
    def __init__(self):
        # 直接实例化检测器
        if detector_type == "yolo_world":
            from .yolo_world_detector import YOLOWorldDetector
            self.detector = YOLOWorldDetector(...)

        # 直接实例化CLIP编码器
        from .clip_encoder import CLIPEncoder
        self.clip_encoder = CLIPEncoder(...)

        # 直接实例化追踪器
        from .instance_tracker import InstanceTracker
        self.tracker = InstanceTracker(...)

    def process_frame(self, image, depth):
        # 直接调用内部方法
        detections = self.detector.detect(image)
        features = self.clip_encoder.encode(image, detections)
        tracked = self.tracker.update(detections)
        return tracked
```

**问题**:
- Node直接依赖具体实现类
- 难以替换检测器（需要修改Node代码）
- 难以单独测试各组件
- 对外使用需要了解内部细节

### 2.2 Semantic Planner 当前结构

```python
# 当前: planner_node.py 直接使用内部类
class SemanticPlannerNode(Node):
    def __init__(self):
        # 直接实例化Goal Resolver
        from .goal_resolver import GoalResolver
        self.goal_resolver = GoalResolver(...)

        # 直接实例化Task Decomposer
        from .task_decomposer import TaskDecomposer
        self.task_decomposer = TaskDecomposer(...)

        # 直接实例化LLM Client
        from .llm_client import create_llm_client
        self.llm_client = create_llm_client(...)

    def handle_instruction(self, instruction):
        # 直接调用内部方法
        tasks = self.task_decomposer.decompose(instruction)
        goal = self.goal_resolver.resolve(tasks[0], scene_graph)
        return goal
```

**问题**:
- Node直接依赖具体实现
- 难以替换Goal Resolver实现
- 难以单独测试
- 对外使用复杂

---

## 3. API重构设计

### 3.1 设计原则

1. **接口与实现分离**: 定义抽象接口，隐藏实现细节
2. **依赖注入**: 通过构造函数注入依赖，而非直接实例化
3. **工厂模式**: 使用工厂创建具体实现
4. **统一错误处理**: 定义统一的异常类型
5. **版本化**: API支持版本管理

### 3.2 目录结构

```
src/
├── semantic_perception/
│   ├── semantic_perception/
│   │   ├── api/                    # 新增：API接口层
│   │   │   ├── __init__.py
│   │   │   ├── perception_api.py   # 感知API接口
│   │   │   ├── detector_api.py     # 检测器API接口
│   │   │   ├── encoder_api.py      # 编码器API接口
│   │   │   ├── tracker_api.py      # 追踪器API接口
│   │   │   └── factory.py          # 工厂类
│   │   ├── impl/                   # 重命名：实现层
│   │   │   ├── yolo_world_detector.py
│   │   │   ├── clip_encoder.py
│   │   │   ├── instance_tracker.py
│   │   │   └── ...
│   │   ├── perception_node.py      # 修改：使用API接口
│   │   └── ...
│   └── ...
│
├── semantic_planner/
│   ├── semantic_planner/
│   │   ├── api/                    # 新增：API接口层
│   │   │   ├── __init__.py
│   │   │   ├── planner_api.py      # 规划API接口
│   │   │   ├── goal_resolver_api.py
│   │   │   ├── task_decomposer_api.py
│   │   │   ├── llm_client_api.py
│   │   │   └── factory.py
│   │   ├── impl/                   # 重命名：实现层
│   │   │   ├── goal_resolver.py
│   │   │   ├── task_decomposer.py
│   │   │   ├── llm_client.py
│   │   │   └── ...
│   │   ├── planner_node.py         # 修改：使用API接口
│   │   └── ...
│   └── ...
│
└── semantic_common/                # 新增：公共API包
    ├── semantic_common/
    │   ├── api/
    │   │   ├── __init__.py
    │   │   ├── scene_graph_api.py  # 场景图API
    │   │   ├── types.py            # 公共类型定义
    │   │   └── exceptions.py       # 公共异常定义
    │   └── ...
    └── ...
```

---

## 4. API接口设计

### 4.1 Semantic Perception API

#### 4.1.1 PerceptionAPI (顶层接口)

```python
# src/semantic_perception/semantic_perception/api/perception_api.py

from abc import ABC, abstractmethod
from typing import List, Optional
from dataclasses import dataclass
import numpy as np

@dataclass
class Detection3D:
    """3D检测结果"""
    id: str
    label: str
    confidence: float
    bbox_2d: List[float]  # [x1, y1, x2, y2]
    position_3d: List[float]  # [x, y, z]
    clip_feature: Optional[np.ndarray] = None


@dataclass
class SceneGraph:
    """场景图"""
    objects: List[Detection3D]
    relations: List[dict]
    timestamp: float


class PerceptionAPI(ABC):
    """
    语义感知API接口

    职责：
    - 处理RGB-D图像
    - 输出3D检测结果和场景图
    """

    @abstractmethod
    def process_frame(
        self,
        rgb_image: np.ndarray,
        depth_image: np.ndarray,
        camera_info: dict,
        transform: Optional[np.ndarray] = None
    ) -> List[Detection3D]:
        """
        处理单帧图像

        Args:
            rgb_image: RGB图像 (H, W, 3)
            depth_image: 深度图像 (H, W)
            camera_info: 相机内参 {fx, fy, cx, cy}
            transform: 相机到世界坐标系的变换矩阵 (4x4)

        Returns:
            3D检测结果列表
        """
        pass

    @abstractmethod
    def get_scene_graph(self) -> SceneGraph:
        """
        获取当前场景图

        Returns:
            场景图对象
        """
        pass

    @abstractmethod
    def reset(self):
        """重置感知系统（清空历史）"""
        pass

    @abstractmethod
    def configure(self, config: dict):
        """
        配置感知系统

        Args:
            config: 配置字典
        """
        pass
```

#### 4.1.2 DetectorAPI (检测器接口)

```python
# src/semantic_perception/semantic_perception/api/detector_api.py

from abc import ABC, abstractmethod
from typing import List
from dataclasses import dataclass
import numpy as np

@dataclass
class Detection2D:
    """2D检测结果"""
    label: str
    confidence: float
    bbox: List[float]  # [x1, y1, x2, y2]


class DetectorAPI(ABC):
    """
    物体检测器API接口

    支持的实现：
    - YOLO-World
    - Grounding DINO
    - 自定义检测器
    """

    @abstractmethod
    def detect(self, image: np.ndarray) -> List[Detection2D]:
        """
        检测图像中的物体

        Args:
            image: RGB图像 (H, W, 3)

        Returns:
            2D检测结果列表
        """
        pass

    @abstractmethod
    def set_classes(self, classes: List[str]):
        """
        设置检测类别（开放词汇检测）

        Args:
            classes: 类别名称列表
        """
        pass

    @abstractmethod
    def get_model_info(self) -> dict:
        """
        获取模型信息

        Returns:
            {name, version, input_size, ...}
        """
        pass
```

#### 4.1.3 EncoderAPI (编码器接口)

```python
# src/semantic_perception/semantic_perception/api/encoder_api.py

from abc import ABC, abstractmethod
from typing import List
import numpy as np

class EncoderAPI(ABC):
    """
    视觉-语言编码器API接口

    支持的实现：
    - CLIP
    - BLIP
    - 自定义编码器
    """

    @abstractmethod
    def encode_image(self, image: np.ndarray) -> np.ndarray:
        """
        编码图像

        Args:
            image: RGB图像 (H, W, 3)

        Returns:
            图像特征向量 (D,)
        """
        pass

    @abstractmethod
    def encode_text(self, text: str) -> np.ndarray:
        """
        编码文本

        Args:
            text: 文本字符串

        Returns:
            文本特征向量 (D,)
        """
        pass

    @abstractmethod
    def compute_similarity(
        self,
        image_features: np.ndarray,
        text_features: np.ndarray
    ) -> float:
        """
        计算图像-文本相似度

        Args:
            image_features: 图像特征 (D,)
            text_features: 文本特征 (D,)

        Returns:
            相似度分数 [0, 1]
        """
        pass
```

#### 4.1.4 Factory (工厂类)

```python
# src/semantic_perception/semantic_perception/api/factory.py

from typing import Optional
from .perception_api import PerceptionAPI
from .detector_api import DetectorAPI
from .encoder_api import EncoderAPI

class PerceptionFactory:
    """感知系统工厂类"""

    @staticmethod
    def create_perception(
        detector_type: str = "yolo_world",
        encoder_type: str = "clip",
        config: Optional[dict] = None
    ) -> PerceptionAPI:
        """
        创建感知系统

        Args:
            detector_type: 检测器类型 (yolo_world | grounding_dino)
            encoder_type: 编码器类型 (clip | blip)
            config: 配置字典

        Returns:
            PerceptionAPI实例
        """
        from ..impl.perception_impl import PerceptionImpl

        detector = PerceptionFactory.create_detector(detector_type, config)
        encoder = PerceptionFactory.create_encoder(encoder_type, config)

        return PerceptionImpl(detector, encoder, config)

    @staticmethod
    def create_detector(
        detector_type: str,
        config: Optional[dict] = None
    ) -> DetectorAPI:
        """创建检测器"""
        if detector_type == "yolo_world":
            from ..impl.yolo_world_detector import YOLOWorldDetector
            return YOLOWorldDetector(config)
        elif detector_type == "grounding_dino":
            from ..impl.grounding_dino_detector import GroundingDINODetector
            return GroundingDINODetector(config)
        else:
            raise ValueError(f"Unknown detector type: {detector_type}")

    @staticmethod
    def create_encoder(
        encoder_type: str,
        config: Optional[dict] = None
    ) -> EncoderAPI:
        """创建编码器"""
        if encoder_type == "clip":
            from ..impl.clip_encoder import CLIPEncoder
            return CLIPEncoder(config)
        else:
            raise ValueError(f"Unknown encoder type: {encoder_type}")
```

---

### 4.2 Semantic Planner API

#### 4.2.1 PlannerAPI (顶层接口)

```python
# src/semantic_planner/semantic_planner/api/planner_api.py

from abc import ABC, abstractmethod
from typing import Optional
from dataclasses import dataclass

@dataclass
class NavigationGoal:
    """导航目标"""
    x: float
    y: float
    z: float
    label: str
    confidence: float
    reasoning: str


@dataclass
class PlannerStatus:
    """规划器状态"""
    state: str  # idle | planning | navigating | completed | failed
    current_task: Optional[str]
    progress: float  # [0, 1]
    message: str


class PlannerAPI(ABC):
    """
    语义规划API接口

    职责：
    - 接收自然语言指令
    - 输出导航目标
    - 管理任务执行
    """

    @abstractmethod
    def plan(
        self,
        instruction: str,
        scene_graph: dict,
        robot_position: Optional[dict] = None
    ) -> NavigationGoal:
        """
        规划导航目标

        Args:
            instruction: 自然语言指令
            scene_graph: 场景图JSON
            robot_position: 机器人当前位置 {x, y, z}

        Returns:
            导航目标
        """
        pass

    @abstractmethod
    def get_status(self) -> PlannerStatus:
        """获取规划器状态"""
        pass

    @abstractmethod
    def cancel(self):
        """取消当前任务"""
        pass

    @abstractmethod
    def reset(self):
        """重置规划器"""
        pass
```

#### 4.2.2 GoalResolverAPI (目标解析接口)

```python
# src/semantic_planner/semantic_planner/api/goal_resolver_api.py

from abc import ABC, abstractmethod
from typing import Optional
from dataclasses import dataclass

@dataclass
class GoalResult:
    """目标解析结果"""
    action: str  # navigate | explore
    target_x: float
    target_y: float
    target_z: float
    target_label: str
    confidence: float
    reasoning: str
    path: str  # fast | slow


class GoalResolverAPI(ABC):
    """
    目标解析API接口

    职责：
    - Fast-Slow双进程目标解析
    - 场景图匹配
    - LLM推理
    """

    @abstractmethod
    def resolve(
        self,
        instruction: str,
        scene_graph: str,
        robot_position: Optional[dict] = None
    ) -> GoalResult:
        """
        解析目标

        Args:
            instruction: 自然语言指令
            scene_graph: 场景图JSON字符串
            robot_position: 机器人位置

        Returns:
            目标解析结果
        """
        pass

    @abstractmethod
    def fast_resolve(
        self,
        instruction: str,
        scene_graph: str,
        robot_position: Optional[dict] = None
    ) -> Optional[GoalResult]:
        """
        Fast Path解析（无LLM）

        Returns:
            GoalResult or None (None表示需要Slow Path)
        """
        pass

    @abstractmethod
    def slow_resolve(
        self,
        instruction: str,
        scene_graph: str,
        robot_position: Optional[dict] = None
    ) -> GoalResult:
        """
        Slow Path解析（使用LLM）

        Returns:
            GoalResult
        """
        pass
```

#### 4.2.3 LLMClientAPI (LLM客户端接口)

```python
# src/semantic_planner/semantic_planner/api/llm_client_api.py

from abc import ABC, abstractmethod
from typing import List, Optional
from dataclasses import dataclass

@dataclass
class LLMMessage:
    """LLM消息"""
    role: str  # system | user | assistant
    content: str


@dataclass
class LLMResponse:
    """LLM响应"""
    content: str
    model: str
    tokens_used: int
    latency_ms: float


class LLMClientAPI(ABC):
    """
    LLM客户端API接口

    支持的实现：
    - OpenAI (GPT-4o, GPT-4o-mini)
    - Claude (Claude 3.5 Sonnet)
    - Qwen (通义千问)
    """

    @abstractmethod
    def chat(
        self,
        messages: List[LLMMessage],
        temperature: float = 0.2,
        max_tokens: int = 1000
    ) -> LLMResponse:
        """
        聊天补全

        Args:
            messages: 消息列表
            temperature: 温度参数
            max_tokens: 最大token数

        Returns:
            LLM响应
        """
        pass

    @abstractmethod
    def get_model_info(self) -> dict:
        """
        获取模型信息

        Returns:
            {provider, model, context_length, ...}
        """
        pass
```

---

## 5. 使用示例

### 5.1 Perception API使用

```python
# 旧方式（直接使用内部类）
from semantic_perception.yolo_world_detector import YOLOWorldDetector
from semantic_perception.clip_encoder import CLIPEncoder

detector = YOLOWorldDetector(config)
encoder = CLIPEncoder()
detections = detector.detect(image)
features = encoder.encode_image(image)

# 新方式（使用API接口）
from semantic_perception.api import PerceptionFactory

# 创建感知系统
perception = PerceptionFactory.create_perception(
    detector_type="yolo_world",
    encoder_type="clip",
    config=config
)

# 处理图像
detections = perception.process_frame(rgb, depth, camera_info, transform)

# 获取场景图
scene_graph = perception.get_scene_graph()
```

### 5.2 Planner API使用

```python
# 旧方式（直接使用内部类）
from semantic_planner.goal_resolver import GoalResolver
from semantic_planner.llm_client import create_llm_client

llm_client = create_llm_client(config)
goal_resolver = GoalResolver(llm_client, config)
result = goal_resolver.resolve(instruction, scene_graph)

# 新方式（使用API接口）
from semantic_planner.api import PlannerFactory

# 创建规划器
planner = PlannerFactory.create_planner(
    goal_resolver_type="fast_slow",
    llm_provider="openai",
    config=config
)

# 规划目标
goal = planner.plan(instruction, scene_graph, robot_position)

# 获取状态
status = planner.get_status()
```

### 5.3 ROS2 Node使用

```python
# 新的perception_node.py
from semantic_perception.api import PerceptionFactory, PerceptionAPI

class SemanticPerceptionNode(Node):
    def __init__(self):
        super().__init__("semantic_perception_node")

        # 通过工厂创建感知系统
        self.perception: PerceptionAPI = PerceptionFactory.create_perception(
            detector_type=self.get_parameter("detector_type").value,
            encoder_type=self.get_parameter("encoder_type").value,
            config=self._load_config()
        )

    def process_callback(self, rgb_msg, depth_msg):
        # 使用API接口
        detections = self.perception.process_frame(
            rgb_image, depth_image, camera_info, transform
        )

        # 发布结果
        self.publish_detections(detections)
```

---

## 6. 实施计划

### 6.1 阶段1：API接口定义（1周）

**任务**:
1. 创建API目录结构
2. 定义所有API接口
3. 定义公共类型和异常
4. 编写API文档

**产出**:
- `src/semantic_perception/semantic_perception/api/`
- `src/semantic_planner/semantic_planner/api/`
- `src/semantic_common/`
- API文档

### 6.2 阶段2：实现层重构（2周）

**任务**:
1. 将现有实现移到`impl/`目录
2. 实现API接口
3. 创建工厂类
4. 更新单元测试

**产出**:
- `src/semantic_perception/semantic_perception/impl/`
- `src/semantic_planner/semantic_planner/impl/`
- 工厂类
- 更新的测试

### 6.3 阶段3：Node层重构（1周）

**任务**:
1. 更新perception_node.py使用API
2. 更新planner_node.py使用API
3. 更新launch文件
4. 集成测试

**产出**:
- 重构的Node代码
- 更新的launch文件
- 集成测试通过

### 6.4 阶段4：文档和示例（1周）

**任务**:
1. 编写API使用文档
2. 创建示例代码
3. 更新CLAUDE.md
4. 更新README

**产出**:
- API使用指南
- 示例代码
- 更新的文档

---

## 7. 优势分析

### 7.1 对内部开发的好处

1. **模块解耦**
   - 各模块独立开发
   - 易于单元测试
   - 降低维护成本

2. **易于扩展**
   - 新增检测器：实现DetectorAPI即可
   - 新增LLM：实现LLMClientAPI即可
   - 不影响现有代码

3. **代码复用**
   - API接口可在多个项目中复用
   - 实现可以独立演进

### 7.2 对外部集成的好处

1. **清晰的接口**
   - 不需要了解内部实现
   - 只需要知道API接口
   - 降低学习成本

2. **稳定的API**
   - 内部实现可以改变
   - API接口保持稳定
   - 向后兼容

3. **易于集成**
   ```python
   # 外部项目使用
   from semantic_perception.api import PerceptionFactory

   perception = PerceptionFactory.create_perception()
   detections = perception.process_frame(rgb, depth, camera_info)
   ```

### 7.3 对论文发表的好处

1. **清晰的系统架构**
   - API层次清晰
   - 易于在论文中描述
   - 提升系统完整性

2. **易于对比实验**
   - 可以轻松替换不同实现
   - 对比Fast Path vs Slow Path
   - 对比不同检测器

---

## 8. 风险和挑战

### 8.1 风险

1. **重构工作量**: 需要4-5周时间
2. **兼容性**: 可能影响现有代码
3. **性能开销**: API层可能引入轻微性能开销
4. **学习曲线**: 团队需要适应新架构

### 8.2 缓解措施

1. **渐进式重构**: 先重构一个模块，验证后再推广
2. **保持兼容**: 保留旧接口一段时间，逐步迁移
3. **性能测试**: 确保API层开销<1%
4. **文档和培训**: 提供详细文档和示例

---

## 9. 总结

### 9.1 核心价值

**API重构将带来**:
- ✅ 统一的接口规范
- ✅ 模块解耦和独立测试
- ✅ 易于维护和扩展
- ✅ 清晰的对外接口
- ✅ 提升系统专业性

### 9.2 推荐方案

**建议采用渐进式重构**:
1. 先重构Semantic Perception（2周）
2. 验证效果后重构Semantic Planner（2周）
3. 最后重构其他模块（1周）

**总时间**: 5周

### 9.3 下一步行动

1. **立即**: 评审本方案，确定是否实施
2. **本周**: 创建API接口定义
3. **下周**: 开始实现层重构
4. **3周后**: 完成第一个模块重构

---

**文档位置**: `docs/03-development/API_REFACTORING_PLAN.md`
**状态**: 📋 方案设计完成，等待评审
**预计工作量**: 5周
**优先级**: 中（不影响论文发表，但提升系统质量）
