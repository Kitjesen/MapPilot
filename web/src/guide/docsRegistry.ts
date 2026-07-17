import architecture from '../../../docs/architecture/README.md?raw'
import buildGuide from '../../../docs/01-getting-started/BUILD_GUIDE.md?raw'
import concepts from '../../../docs/02-concepts/README.md?raw'
import current from '../../../docs/CURRENT.md?raw'
import deploymentGuide from '../../../docs/04-deployment/WEB_GUIDE.md?raw'
import development from '../../../docs/03-development/README.md?raw'
import docsHome from '../../../docs/README.md?raw'
import gatewayRest from '../../../docs/api/gateway_rest.md?raw'
import gettingStarted from '../../../docs/01-getting-started/README.md?raw'
import integrations from '../../../docs/09-integrations/README.md?raw'
import knownGaps from '../../../docs/known_gaps.md?raw'
import mapServiceContract from '../../../docs/architecture/MAP_SERVICE_CONTRACT.md?raw'
import mcpTools from '../../../docs/api/mcp_tools.md?raw'
import moduleServiceBoundary from '../../../docs/architecture/MODULE_SERVICE_BOUNDARY.md?raw'
import navigationComputeContract from '../../../docs/architecture/NAVIGATION_COMPUTE_CONTRACT.md?raw'
import operations from '../../../docs/06-operations/README.md?raw'
import quickStart from '../../../docs/QUICKSTART.md?raw'
import reference from '../../../docs/08-reference/README.md?raw'
import runtimeBus from '../../../docs/architecture/LINGTU_RUNTIME_BUS_DECISION.md?raw'
import safety from '../../../docs/10-safety/README.md?raw'
import systemDesign from '../../../docs/architecture/SYSTEM_DESIGN.md?raw'
import taskGuides from '../../../docs/05-guides/README.md?raw'
import validationGuide from '../../../docs/07-testing/WEB_GUIDE.md?raw'
import troubleshooting from '../../../docs/03-development/TROUBLESHOOTING.md?raw'

export type DocumentationStatus = 'Current' | 'Reference' | 'Evidence' | 'Plan' | 'Historical'

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
  contentSourcePath?: string
  hidden?: boolean
}

export type GuideGroup = {
  label: string
  ids: string[]
}

const verifiedToday = '2026-07-15'

export const guideDocuments: GuideDocument[] = [
  {
    id: 'home',
    title: 'LingTu 文档',
    description: '面向四足机器人自主导航的产品、开发与现场操作文档。',
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
    id: 'quick-start',
    title: 'Quick Start',
    description: '在本地、仿真或受监督现场会话中选择正确的首个入口。',
    sourcePath: 'docs/QUICKSTART.md',
    content: quickStart,
    status: 'Current',
    audience: '开发者、集成者与受监督的机器人操作员',
    runsOn: '本地、仿真和受支持的现场机器人',
    operation: 'Motion-capable',
    lastVerified: verifiedToday,
    group: '开始使用',
  },
  {
    id: 'getting-started',
    title: '开始使用',
    description: '按本地、仿真与现场边界分阶段认识并运行 LingTu。',
    sourcePath: 'docs/01-getting-started/README.md',
    content: gettingStarted,
    status: 'Current',
    audience: '首次使用 LingTu 的开发者和集成者',
    runsOn: '本地、仿真和现场',
    operation: 'Read-only',
    lastVerified: verifiedToday,
    group: '开始使用',
  },
  {
    id: 'build-guide',
    title: '构建与部署准备',
    description: '建立可重复的 Python、原生组件和目标环境。',
    sourcePath: 'docs/01-getting-started/BUILD_GUIDE.md',
    content: buildGuide,
    status: 'Current',
    audience: '开发者与部署集成者',
    runsOn: '开发主机与受支持的目标系统',
    operation: 'State-changing',
    lastVerified: verifiedToday,
    group: '开始使用',
  },
  {
    id: 'concepts',
    title: '核心概念',
    description: '理解 Module-First、Blueprint、端口、显式连线和传输边界。',
    sourcePath: 'docs/02-concepts/README.md',
    content: concepts,
    status: 'Current',
    audience: '开发者和架构集成者',
    runsOn: '所有 LingTu 环境',
    operation: 'Read-only',
    lastVerified: verifiedToday,
    group: '理解 LingTu',
  },
  {
    id: 'architecture',
    title: '架构合同',
    description: '查阅当前架构、运行时和服务边界的权威入口。',
    sourcePath: 'docs/architecture/README.md',
    content: architecture,
    status: 'Reference',
    audience: '架构师、后端和原生端开发者',
    runsOn: '所有 LingTu 环境',
    operation: 'Read-only',
    lastVerified: verifiedToday,
    group: '理解 LingTu',
  },
  {
    id: 'safety',
    title: '安全与执行边界',
    description: '识别只读、状态变更和可能产生机器人运动的操作。',
    sourcePath: 'docs/10-safety/README.md',
    content: safety,
    status: 'Current',
    audience: '所有操作员、开发者与集成者',
    runsOn: '本地、仿真和现场',
    operation: 'Motion-capable',
    lastVerified: verifiedToday,
    group: '理解 LingTu',
  },
  {
    id: 'development',
    title: '开发 LingTu',
    description: '以小而可验证的改动扩展模块、栈和后端。',
    sourcePath: 'docs/03-development/README.md',
    content: development,
    status: 'Current',
    audience: 'LingTu 开发者',
    runsOn: '本地开发主机',
    operation: 'Read-only',
    lastVerified: verifiedToday,
    group: '开发与集成',
  },
  {
    id: 'troubleshooting',
    title: '排障',
    description: '从症状开始，选择不扩大风险的诊断和恢复路径。',
    sourcePath: 'docs/03-development/TROUBLESHOOTING.md',
    content: troubleshooting,
    status: 'Current',
    audience: '开发者、集成者与操作员',
    runsOn: '本地、仿真和现场',
    operation: 'Read-only',
    lastVerified: verifiedToday,
    group: '开发与集成',
  },
  {
    id: 'integrations',
    title: 'SDK、REST 与 MCP 集成',
    description: '在明确鉴权、租约和动作级别的前提下接入 LingTu。',
    sourcePath: 'docs/09-integrations/README.md',
    content: integrations,
    status: 'Current',
    audience: '应用开发者与系统集成者',
    runsOn: 'Gateway 可用的本地、仿真或现场环境',
    operation: 'Motion-capable',
    lastVerified: verifiedToday,
    group: '开发与集成',
  },
  {
    id: 'task-guides',
    title: '按任务使用 LingTu',
    description: '从建图、导航、语义任务、探索到客户端集成，按目标选择流程。',
    sourcePath: 'docs/05-guides/README.md',
    content: taskGuides,
    status: 'Current',
    audience: '开发者、集成者与受监督操作员',
    runsOn: '本地、仿真和现场',
    operation: 'Motion-capable',
    lastVerified: verifiedToday,
    group: '任务与运行',
  },
  {
    id: 'operations',
    title: '运行与运维',
    description: '安全地观察、诊断、恢复和维护正在运行的系统。',
    sourcePath: 'docs/06-operations/README.md',
    content: operations,
    status: 'Current',
    audience: '现场操作员与值守工程师',
    runsOn: '部署后的 LingTu 系统',
    operation: 'Motion-capable',
    lastVerified: verifiedToday,
    group: '任务与运行',
  },
  {
    id: 'deployment',
    title: '部署',
    description: '为目标机器人准备、部署并验证 LingTu 运行环境。',
    sourcePath: 'docs/04-deployment/README.md',
    contentSourcePath: 'docs/04-deployment/WEB_GUIDE.md',
    content: deploymentGuide,
    status: 'Reference',
    audience: '部署工程师和现场集成者',
    runsOn: '受支持的现场目标系统',
    operation: 'State-changing',
    lastVerified: verifiedToday,
    group: '任务与运行',
  },
  {
    id: 'reference',
    title: '参考',
    description: '查找 CLI、配置、接口和架构引用的入口。',
    sourcePath: 'docs/08-reference/README.md',
    content: reference,
    status: 'Reference',
    audience: '开发者、集成者和操作员',
    runsOn: '取决于所查阅的接口',
    operation: 'Read-only',
    lastVerified: verifiedToday,
    group: '参考与证据',
  },
  {
    id: 'testing',
    title: '测试与验证证据',
    description: '了解本地、仿真和现场验证分别证明什么。',
    sourcePath: 'docs/07-testing/README.md',
    contentSourcePath: 'docs/07-testing/WEB_GUIDE.md',
    content: validationGuide,
    status: 'Evidence',
    audience: '开发者、验证工程师与发布负责人',
    runsOn: '本地、仿真和现场',
    operation: 'Read-only',
    lastVerified: verifiedToday,
    group: '参考与证据',
  },
  {
    id: 'current',
    title: '当前文档状态',
    description: '分辨当前合同、参考、验证证据、计划与历史材料。',
    sourcePath: 'docs/CURRENT.md',
    content: current,
    status: 'Reference',
    audience: '所有文档读者',
    runsOn: '不适用',
    operation: 'Read-only',
    lastVerified: verifiedToday,
    group: '参考与证据',
  },
  {
    id: 'known-gaps',
    title: '已知限制',
    description: '查看当前能力的边界、已知缺口与不应做出的推断。',
    sourcePath: 'docs/known_gaps.md',
    content: knownGaps,
    status: 'Reference',
    audience: '评估 LingTu 的开发者与集成者',
    runsOn: '取决于具体能力',
    operation: 'Read-only',
    lastVerified: verifiedToday,
    group: '参考与证据',
  },
  {
    id: 'system-design',
    title: '系统设计',
    description: '当前层级、所有权和系统边界的架构合同。',
    sourcePath: 'docs/architecture/SYSTEM_DESIGN.md',
    content: systemDesign,
    status: 'Reference',
    audience: '架构师和系统集成者',
    runsOn: '所有 LingTu 环境',
    operation: 'Read-only',
    lastVerified: verifiedToday,
    group: '关联参考',
    hidden: true,
  },
  {
    id: 'runtime-bus',
    title: 'Module-First Runtime Bus',
    description: 'Module、Blueprint、Port、Wire 与传输边界的运行时合同。',
    sourcePath: 'docs/architecture/LINGTU_RUNTIME_BUS_DECISION.md',
    content: runtimeBus,
    status: 'Reference',
    audience: '框架开发者和模块作者',
    runsOn: '所有 LingTu 环境',
    operation: 'Read-only',
    lastVerified: verifiedToday,
    group: '关联参考',
    hidden: true,
  },
  {
    id: 'module-service-boundary',
    title: 'Module / Service 边界',
    description: '将模块图、进程服务与外部适配器分开的当前规则。',
    sourcePath: 'docs/architecture/MODULE_SERVICE_BOUNDARY.md',
    content: moduleServiceBoundary,
    status: 'Reference',
    audience: '架构师和后端开发者',
    runsOn: '所有 LingTu 环境',
    operation: 'Read-only',
    lastVerified: verifiedToday,
    group: '关联参考',
    hidden: true,
  },
  {
    id: 'map-service-contract',
    title: '地图服务合同',
    description: '保存地图、派生工件、校验与地图来源的一致性规则。',
    sourcePath: 'docs/architecture/MAP_SERVICE_CONTRACT.md',
    content: mapServiceContract,
    status: 'Reference',
    audience: '地图、导航和部署集成者',
    runsOn: '地图构建与导航环境',
    operation: 'State-changing',
    lastVerified: verifiedToday,
    group: '关联参考',
    hidden: true,
  },
  {
    id: 'navigation-compute-contract',
    title: '导航计算合同',
    description: '全局规划、局部执行与安全检查之间的数据流约定。',
    sourcePath: 'docs/architecture/NAVIGATION_COMPUTE_CONTRACT.md',
    content: navigationComputeContract,
    status: 'Reference',
    audience: '导航开发者和系统集成者',
    runsOn: '仿真和现场导航环境',
    operation: 'Motion-capable',
    lastVerified: verifiedToday,
    group: '关联参考',
    hidden: true,
  },
  {
    id: 'gateway-rest-reference',
    title: 'Gateway REST 清单',
    description: '仓库生成的 Gateway 路由清单；运行时 OpenAPI 仍是具体目标的规范。',
    sourcePath: 'docs/api/gateway_rest.md',
    content: gatewayRest,
    status: 'Reference',
    audience: 'REST 客户端作者',
    runsOn: 'Gateway 可用的环境',
    operation: 'Motion-capable',
    lastVerified: verifiedToday,
    group: '关联参考',
    hidden: true,
  },
  {
    id: 'mcp-tool-reference',
    title: 'MCP 工具清单',
    description: '仓库生成的 MCP 工具清单；活动 Profile 决定真实可用工具。',
    sourcePath: 'docs/api/mcp_tools.md',
    content: mcpTools,
    status: 'Reference',
    audience: 'MCP 客户端和代理作者',
    runsOn: 'MCPServerModule 已加载的环境',
    operation: 'Motion-capable',
    lastVerified: verifiedToday,
    group: '关联参考',
    hidden: true,
  },
]

export const guideGroups: GuideGroup[] = [
  { label: '开始使用', ids: ['quick-start', 'getting-started', 'build-guide'] },
  { label: '理解 LingTu', ids: ['concepts', 'architecture', 'safety'] },
  { label: '开发与集成', ids: ['development', 'troubleshooting', 'integrations'] },
  { label: '任务与运行', ids: ['task-guides', 'operations', 'deployment'] },
  { label: '参考与证据', ids: ['reference', 'testing', 'current', 'known-gaps'] },
]

export const documentsById = new Map(guideDocuments.map((document) => [document.id, document]))

const documentSourceEntries: Array<[string, GuideDocument]> = guideDocuments.flatMap((document) => {
  const entries: Array<[string, GuideDocument]> = [[document.sourcePath, document]]
  if (document.contentSourcePath) {
    entries.push([document.contentSourcePath, document])
  }
  return entries
})

export const documentsBySourcePath = new Map(documentSourceEntries)

export function findDocument(id: string): GuideDocument {
  return documentsById.get(id) ?? guideDocuments[0]
}

export function findDocumentBySourcePath(sourcePath: string): GuideDocument | undefined {
  return documentsBySourcePath.get(sourcePath)
}
