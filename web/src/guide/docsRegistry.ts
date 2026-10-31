import api from '../../../docs/api.md?raw'
import architecture from '../../../docs/architecture.md?raw'
import development from '../../../docs/development.md?raw'
import gettingStarted from '../../../docs/getting-started.md?raw'
import docsHome from '../../../docs/README.md?raw'
import operations from '../../../docs/operations.md?raw'
import product from '../../../docs/product.md?raw'
import roadmap from '../../../docs/roadmap.md?raw'
import runtime from '../../../docs/runtime.md?raw'
import simulation from '../../../docs/simulation.md?raw'
import testing from '../../../docs/testing.md?raw'

export type DocumentationStatus = 'Current' | 'Reference' | 'Evidence' | 'Plan'

export type OperationBoundary = 'Read-only' | 'State-changing' | 'Motion-capable'

export type GuideDocument = {
  id: string
  title: string
  description: string
  sourcePath: string
  content: string
  status: DocumentationStatus
  audience: string
  runsOn: string
  operation: OperationBoundary
  group: string
}

export type GuideGroup = {
  label: string
  ids: string[]
}

export const guideDocuments: GuideDocument[] = [
  {
    id: 'home',
    title: 'LingTu 文档',
    description: '按产品使用、开发集成和验证交付组织的当前手册。',
    sourcePath: 'docs/README.md',
    content: docsHome,
    status: 'Current',
    audience: '开发者、集成者与机器人操作员',
    runsOn: '本地、仿真和受支持的现场机器人',
    operation: 'Read-only',
    group: 'home',
  },
  {
    id: 'product',
    title: '产品说明',
    description: '产品用途、操作流程、机器人支持与尚待验收的交付范围。',
    sourcePath: 'docs/product.md',
    content: product,
    status: 'Current',
    audience: '产品负责人、操作员与交付工程师',
    runsOn: '真实与仿真环境',
    operation: 'Read-only',
    group: '产品与使用',
  },
  {
    id: 'getting-started',
    title: '开始使用',
    description: '安装依赖、解析 Product，并从本地或仿真完成第一次运行。',
    sourcePath: 'docs/getting-started.md',
    content: gettingStarted,
    status: 'Current',
    audience: '首次使用 LingTu 的开发者和集成者',
    runsOn: '本地、仿真和受监督的现场机器人',
    operation: 'Motion-capable',
    group: '产品与使用',
  },
  {
    id: 'architecture',
    title: '架构',
    description: '仓库布局、分层、所有权和跨进程边界的当前合同。',
    sourcePath: 'docs/architecture.md',
    content: architecture,
    status: 'Reference',
    audience: '架构师、后端和原生端开发者',
    runsOn: '所有 LingTu 环境',
    operation: 'Read-only',
    group: '开发与集成',
  },
  {
    id: 'runtime',
    title: '运行时',
    description: 'ProductControl、RunPlan、DDS、地图和导航数据流合同。',
    sourcePath: 'docs/runtime.md',
    content: runtime,
    status: 'Reference',
    audience: '运行时、导航和驱动开发者',
    runsOn: '真实与仿真环境',
    operation: 'Motion-capable',
    group: '开发与集成',
  },
  {
    id: 'simulation',
    title: '仿真',
    description: '仿真工作区、Package、Session、适配器和验收边界。',
    sourcePath: 'docs/simulation.md',
    content: simulation,
    status: 'Current',
    audience: '仿真、算法和验证开发者',
    runsOn: 'Windows 与 Linux/WSL 的原生 MuJoCo；可选 UE 展示',
    operation: 'State-changing',
    group: '验证与交付',
  },
  {
    id: 'development',
    title: '开发',
    description: '代码放置、构建、测试、参数调整和提交约定。',
    sourcePath: 'docs/development.md',
    content: development,
    status: 'Current',
    audience: 'LingTu 开发者',
    runsOn: '本地开发主机',
    operation: 'State-changing',
    group: '开发与集成',
  },
  {
    id: 'operations',
    title: '现场操作',
    description: '建图、保存重载、定位、导航、人工接管、停止与诊断。',
    sourcePath: 'docs/operations.md',
    content: operations,
    status: 'Current',
    audience: '部署工程师、现场操作员与值守工程师',
    runsOn: '受支持的现场机器人与部署主机',
    operation: 'Motion-capable',
    group: '产品与使用',
  },
  {
    id: 'api',
    title: '接口参考',
    description: 'Gateway REST、MCP 与外部集成的入口和生成清单。',
    sourcePath: 'docs/api.md',
    content: api,
    status: 'Reference',
    audience: 'SDK、REST 与 MCP 客户端作者',
    runsOn: 'Gateway 或 MCPServerModule 可用的环境',
    operation: 'Motion-capable',
    group: '开发与集成',
  },
  {
    id: 'testing',
    title: '测试与证据',
    description: '本地、仿真、无运动和受监督运动证据各自证明什么。',
    sourcePath: 'docs/testing.md',
    content: testing,
    status: 'Evidence',
    audience: '开发者、验证工程师与发布负责人',
    runsOn: '本地、仿真和现场',
    operation: 'Motion-capable',
    group: '验证与交付',
  },
  {
    id: 'roadmap',
    title: '路线图',
    description: '当前交付任务、未完成工作、验收进展与发布条件。',
    sourcePath: 'docs/roadmap.md',
    content: roadmap,
    status: 'Plan',
    audience: '维护者、架构师与产品负责人',
    runsOn: '不适用',
    operation: 'Read-only',
    group: '验证与交付',
  },
]

export const guideGroups: GuideGroup[] = [
  { label: '产品与使用', ids: ['product', 'getting-started', 'operations'] },
  { label: '开发与集成', ids: ['architecture', 'runtime', 'development', 'api'] },
  { label: '验证与交付', ids: ['simulation', 'testing', 'roadmap'] },
]

export const documentsById = new Map(guideDocuments.map((document) => [document.id, document]))

export const documentsBySourcePath = new Map(
  guideDocuments.map((document): [string, GuideDocument] => [document.sourcePath, document]),
)

export function findDocument(id: string): GuideDocument {
  return documentsById.get(id) ?? guideDocuments[0]
}

export function findDocumentBySourcePath(sourcePath: string): GuideDocument | undefined {
  return documentsBySourcePath.get(sourcePath)
}
