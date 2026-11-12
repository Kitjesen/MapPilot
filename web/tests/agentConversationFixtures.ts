import { conversationToolNames } from '../src/services/agentConversation.ts'
import * as sensors from '../src/services/sensors.ts'
import * as botProfile from '../src/services/botProfile.ts'
import * as botMotion from '../src/components/agentConversation/botMotion.ts'
import * as botShapes from '../src/services/botShapes.ts'
import * as modelMentions from '../src/components/agentConversation/modelMentions.ts'
import * as markdownBlocks from '../src/components/agentConversation/markdownStreamBlocks.ts'
import { createRequire } from 'node:module'
import { readFileSync } from 'node:fs'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'

const require = createRequire(import.meta.url)
const mapsSource = ts.transpileModule(readFileSync(new URL('../src/components/agentConversation/SavedMapsResult.tsx', import.meta.url), 'utf8'), {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
}).outputText
const sensorSource = ts.transpileModule(readFileSync(new URL('../src/components/agentConversation/SensorResult.tsx', import.meta.url), 'utf8'), {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
}).outputText
const cloudSource = ts.transpileModule(readFileSync(new URL('../src/components/agentConversation/CloudPreview.tsx', import.meta.url), 'utf8'), {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
}).outputText
const source = ts.transpileModule(readFileSync(new URL('../src/components/agentConversation/ToolActivity.tsx', import.meta.url), 'utf8'), {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
}).outputText
const questionSource = ts.transpileModule(readFileSync(new URL('../src/components/agentConversation/QuestionCard.tsx', import.meta.url), 'utf8'), {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
}).outputText
const botSource = ts.transpileModule(readFileSync(new URL('../src/components/agentConversation/Bot.tsx', import.meta.url), 'utf8'), {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
}).outputText
const addSource = ts.transpileModule(readFileSync(new URL('../src/components/agentConversation/ComposerAdd.tsx', import.meta.url), 'utf8'), {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
}).outputText
export function conversationDependency(name: string) {
  if (name.endsWith('/ObservationCard')) {
    const exports: Record<string, unknown> = {}
    const code = ts.transpileModule(readFileSync(new URL('../src/components/agentConversation/ObservationCard.tsx', import.meta.url), 'utf8'), { compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX } }).outputText
    runInNewContext(code, { exports, require: (dep: string) => dep.endsWith('.css') ? { default: new Proxy({}, { get: (_target, key) => key }) } : require(dep) })
    return exports
  }
  if (name.endsWith('/LiveCameraCard')) {
    const exports: Record<string, unknown> = {}
    const code = ts.transpileModule(readFileSync(new URL('../src/components/agentConversation/LiveCameraCard.tsx', import.meta.url), 'utf8'), {
      compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
    }).outputText
    runInNewContext(code, { exports, require: (dependency: string) => dependency.endsWith('/RecordingLiveCamera') ? {}
      : dependency.endsWith('.css') ? { default: new Proxy({}, { get: (_target, key) => key }) } : require(dependency) })
    return exports
  }
  if (name.endsWith('/SavedMapCard')) {
    const exports: Record<string, unknown> = {}
    const code = ts.transpileModule(readFileSync(new URL('../src/components/agentConversation/SavedMapCard.tsx', import.meta.url), 'utf8'), {
      compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
    }).outputText
    runInNewContext(code, { exports, require: (dependency: string) => dependency.endsWith('/services/mapThumbnail.ts') ? {}
      : dependency.endsWith('.css') ? { default: new Proxy({}, { get: (_target, key) => key }) } : require(dependency) })
    return exports
  }
  if (name.endsWith('/stageshow/motion/Collapse')) {
    const exports: Record<string, unknown> = {}
    const code = ts.transpileModule(readFileSync(new URL('../src/components/stageshow/motion/Collapse.tsx', import.meta.url), 'utf8'), {
      compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
    }).outputText
    runInNewContext(code, { exports, require: (dependency: string) => dependency.endsWith('.css') ? {} : require(dependency) })
    return exports
  }
  if (name.endsWith('/agentConversation/PlanCard.tsx')) {
    const exports: Record<string, unknown> = {}
    const code = ts.transpileModule(readFileSync(new URL('../src/components/agentConversation/PlanCard.tsx', import.meta.url), 'utf8'), {
      compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
    }).outputText
    runInNewContext(code, { exports, require: (dependency: string) => dependency.endsWith('.css')
      ? { default: new Proxy({}, { get: (_target, key) => key }) } : require(dependency) })
    return exports
  }
  if (name.endsWith('/stageshow/motion/Roll')) {
    const exports: Record<string, unknown> = {}
    const code = ts.transpileModule(readFileSync(new URL('../src/components/stageshow/motion/Roll.tsx', import.meta.url), 'utf8'), {
      compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
    }).outputText
    runInNewContext(code, { exports, require: (dependency: string) => dependency.endsWith('/reduced-motion')
      ? { useReducedMotion: () => true } : dependency.endsWith('.css') ? {} : require(dependency) })
    return exports
  }
  if (name.endsWith('/agentConversation/markdownStreamBlocks.ts')) return markdownBlocks
  if (name.endsWith('/agentConversation/QueuedMessages')) {
    const exports: Record<string, unknown> = {}
    const code = ts.transpileModule(readFileSync(new URL('../src/components/agentConversation/QueuedMessages.tsx', import.meta.url), 'utf8'), {
      compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
    }).outputText
    runInNewContext(code, { exports, require: (dependency: string) => dependency.endsWith('.css')
      ? { default: new Proxy({}, { get: (_target, key) => key }) } : require(dependency) })
    return exports
  }
  if (name.endsWith('/hooks/useSensorMonitor.ts')) return { useSensorMonitor: () => null }
  if (name.endsWith('/hooks/useImuMonitor.ts')) return { useImuMonitor: () => null }
  if (name.endsWith('/agentConversation/ComposerAdd.tsx')) {
    const exports: Record<string, unknown> = {}
    runInNewContext(addSource, { exports, require: (dependency: string) => dependency.endsWith('/hooks/useSensorMonitor.ts') ? { useSensorMonitor: () => null } : dependency.endsWith('/hooks/useImuMonitor.ts') ? { useImuMonitor: () => null } : dependency.endsWith('.css')
      ? { default: new Proxy({}, { get: (_target, key) => key }) } : require(dependency) })
    return exports
  }
  if (name.endsWith('/agentConversation/modelMentions.ts')) return modelMentions
  if (name.endsWith('/agentConversation/QuestionCard.tsx')) {
    const exports: Record<string, unknown> = {}
    runInNewContext(questionSource, { exports, require: (dependency: string) => dependency.endsWith('.css')
      ? { default: new Proxy({}, { get: (_target, key) => key }) } : require(dependency) })
    return exports
  }
  if (name.endsWith('/services/sensors.ts')) return sensors
  if (name.endsWith('/services/botProfile.ts')) return botProfile
  if (name.endsWith('/agentConversation/BotStudio')) return { BotStudio: () => null }
  if (name.endsWith('/agentConversation/ModelConnections')) return { ModelConnections: () => null }
  if (name.endsWith('/agentConversation/Bot.tsx')) {
    const exports: Record<string, unknown> = {}
    runInNewContext(botSource, { exports, require: (dependency: string) => {
      if (dependency.endsWith('/botMotion.ts')) return botMotion
      if (dependency.endsWith('/services/botProfile.ts')) return botProfile
      if (dependency.endsWith('/services/botShapes.ts')) return botShapes
      if (dependency.endsWith('/GrokAvatar.tsx')) {
        const result: Record<string, unknown> = {}
        const code = ts.transpileModule(readFileSync(new URL('../src/components/agentConversation/GrokAvatar.tsx', import.meta.url), 'utf8'), {
          compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
        }).outputText
        runInNewContext(code, { exports: result, require: (name: string) => name.includes('/vendor/grok-avatar/')
          ? require('../src/vendor/grok-avatar/src/index.mjs') : require(name) })
        return result
      }
      if (dependency.endsWith('.css')) return { default: new Proxy({}, { get: (_target, key) => key }) }
      return require(dependency)
    } })
    return exports
  }
  if (name.endsWith('/services/agentConversation.ts')) return { conversationToolNames }
  if (name.endsWith('/hooks/useMapSettings.ts')) return { useMapSettings: () => null }
  if (name.endsWith('/agentConversation/ToolActivity.tsx')) {
    const exports: Record<string, unknown> = {}
    runInNewContext(source, { exports, require: (dependency: string) => {
      if (dependency.endsWith('/ObservationCard')) return conversationDependency(dependency)
      if (dependency.endsWith('/LiveCameraCard')) return conversationDependency(dependency)
      if (dependency.endsWith('/SavedMapCard')) return conversationDependency(dependency)
      if (dependency.includes('/stageshow/motion/')) return conversationDependency(dependency)
      if (dependency.endsWith('/SavedMapsResult')) {
        const result: Record<string, unknown> = {}
        runInNewContext(mapsSource, { exports: result, require: (dep: string) => dep.endsWith('/SavedMapCard') ? conversationDependency(dep) : dep.endsWith('.css')
          ? { default: new Proxy({}, { get: (_target, key) => key }) } : require(dep) })
        return result
      }
      if (dependency.endsWith('/SensorResult')) {
        const result: Record<string, unknown> = {}
        runInNewContext(sensorSource, { exports: result, require: (dep: string) => dep.endsWith('/LiveCameraCard') ? conversationDependency(dep) : dep.endsWith('/services/sensors.ts') ? sensors : dep.endsWith('/hooks/useSensorMonitor.ts') ? { useSensorMonitor: () => null } : dep.endsWith('/hooks/useImuMonitor.ts') ? { useImuMonitor: () => null } : dep.endsWith('.css')
          ? { default: new Proxy({}, { get: (_target, key) => key }) } : require(dep) })
        return result
      }
      if (dependency.endsWith('/CloudPreview')) {
        const result: Record<string, unknown> = {}
        runInNewContext(cloudSource, { exports: result, require: (dep: string) => {
          if (dep.endsWith('/hooks/useMapSettings.ts')) return { useMapSettings: () => null }
          if (dep.endsWith('/services/api.ts') || dep.endsWith('/services/mapEdit.ts')) return {}
          if (dep.endsWith('.css')) return { default: new Proxy({}, { get: (_target, key) => key }) }
          return require(dep)
        } })
        return result
      }
      if (dependency.endsWith('/Modal')) return { ModalShell: () => null }
      if (dependency.endsWith('/services/agentConversation.ts')) return { conversationToolNames }
      if (dependency.endsWith('/hooks/useMapSettings.ts')) return { useMapSettings: () => null }
      if (dependency.endsWith('/services/api.ts') || dependency.endsWith('/services/mapEdit.ts')) return {}
      if (dependency.endsWith('.css')) return { default: new Proxy({}, { get: (_target, key) => key }) }
      return require(dependency)
    } })
    return exports
  }
  return undefined
}
