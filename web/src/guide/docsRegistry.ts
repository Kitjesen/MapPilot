import api from '../../../docs/api.md?raw'
import architecture from '../../../docs/architecture.md?raw'
import development from '../../../docs/development.md?raw'
import gettingStarted from '../../../docs/getting-started.md?raw'
import docsHome from '../../../docs/README.md?raw'
import operations from '../../../docs/operations.md?raw'
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
  lastVerified: string
  group: string
}

export type GuideGroup = {
  label: string
  ids: string[]
}

const verifiedToday = '2026-09-03'

export const guideDocuments: GuideDocument[] = [
  {
    id: 'home',
    title: 'LingTu 文档',
    description: '十篇维护中的文档，覆盖上手、架构、运行、开发、运维和验证。',
    sourcePath: 'docs/README.md',
    content: docsHome,
    status: 'Current',
    audience: '开发者、集成者与机器人操作员',
    runsOn: '本地、仿真和受支持的现场机器人',
    operation: 'Read-only',
    lastVerified: verifiedToday,
    group: 'home',
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
    lastVerified: verifiedToday,
    group: '开始使用',
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
    lastVerified: verifiedToday,
    group: '理解系统',
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
    lastVerified: verifiedToday,
    group: '理解系统',
  },
  {
    id: 'simulation',
    title: '仿真',
    description: '仿真工作区、Package、Session、适配器和验收边界。',
    sourcePath: 'docs/simulation.md',
    content: simulation,
    status: 'Current',
    audience: '仿真、算法和验证开发者',
    runsOn: 'MuJoCo、Gazebo 与 Windows SimStudio',
    operation: 'State-changing',
    lastVerified: verifiedToday,
    group: '理解系统',
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
    lastVerified: verifiedToday,
    group: '构建与运行',
  },
  {
    id: 'operations',
    title: '部署与运维',
    description: '发布、安装、回滚、诊断和受监督现场操作。',
    sourcePath: 'docs/operations.md',
    content: operations,
    status: 'Current',
    audience: '部署工程师、现场操作员与值守工程师',
    runsOn: '受支持的现场机器人与部署主机',
    operation: 'Motion-capable',
    lastVerified: verifiedToday,
    group: '构建与运行',
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
    lastVerified: verifiedToday,
    group: '参考与计划',
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
    lastVerified: verifiedToday,
    group: '参考与计划',
  },
  {
    id: 'roadmap',
    title: '路线图',
    description: '当前能力缺口、优先级、许可边界和下一步工作。',
    sourcePath: 'docs/roadmap.md',
    content: roadmap,
    status: 'Plan',
    audience: '维护者、架构师与产品负责人',
    runsOn: '不适用',
    operation: 'Read-only',
    lastVerified: verifiedToday,
    group: '参考与计划',
  },
]

export const guideGroups: GuideGroup[] = [
  { label: '开始使用', ids: ['getting-started'] },
  { label: '理解系统', ids: ['architecture', 'runtime', 'simulation'] },
  { label: '构建与运行', ids: ['development', 'operations'] },
  { label: '参考与计划', ids: ['api', 'testing', 'roadmap'] },
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
