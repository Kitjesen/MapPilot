const marks: Record<string, string> = {
  claude: 'anthropic.png', codex: 'openai.svg', chatgpt: 'openai.svg', openai: 'openai.svg', gpt: 'openai.svg',
  kimi: 'moonshot.ico', qwen: 'qwen.png', '通义千问': 'qwen.png', gemini: 'google-gemini.png', google: 'google-gemini.png',
  deepseek: 'deepseek.ico', grok: 'grok.ico', mistral: 'mistral.ico', meta: 'meta.ico', llama: 'meta.ico',
  perplexity: 'perplexity.ico', minimax: 'minimax.ico', glm: 'zhipu.png', zhipu: 'zhipu.png', '智谱': 'zhipu.png',
  cohere: 'cohere.ico', ernie: 'baidu.ico', '文心一言': 'baidu.ico',
}
const labels: Record<string, string> = { claude: 'Anthropic', codex: 'OpenAI', openai: 'OpenAI', gpt: 'OpenAI', chatgpt: 'OpenAI',
  kimi: 'Moonshot', qwen: 'Alibaba', '通义千问': 'Alibaba', gemini: 'Google', google: 'Google', deepseek: 'DeepSeek', grok: 'xAI',
  mistral: 'Mistral', meta: 'Meta', llama: 'Meta', perplexity: 'Perplexity', minimax: 'MiniMax', glm: 'Zhipu AI', zhipu: 'Zhipu AI',
  '智谱': 'Zhipu AI', cohere: 'Cohere', ernie: 'Baidu', '文心一言': 'Baidu' }

// Decorate only text in our already-escaped Markdown output, never tags, URLs or code.
export function addModelMarks(html: string): string {
  let codeDepth = 0
  return html.split(/(<[^>]+>)/g).map(part => {
    if (part.startsWith('<')) {
      if (/^<(?:pre|code)(?:\s|>)/i.test(part)) codeDepth++
      if (/^<\/(?:pre|code)>/i.test(part)) codeDepth--
      return part
    }
    if (codeDepth) return part
    return part.split(/((?:https?:\/\/|mailto:)[^\s]+)/gi).map(segment => /^(?:https?:\/\/|mailto:)/i.test(segment) ? segment
      : segment.replace(/(?<![\w/\\.@-])(Claude|Codex|ChatGPT|OpenAI|GPT(?:-[\d]+(?:\.[\d]+)*)?|Kimi|Qwen(?:\d+(?:\.\d+)*)?|通义千问|Gemini|Google|DeepSeek|Grok|Mistral|Meta|Llama|Perplexity|MiniMax|GLM|Zhipu|智谱|Cohere|Ernie|文心一言)(?![\w/\\.@-])/gi, name => {
      const key = name.toLowerCase().replace(/[-\d].*$/, '')
      const mark = marks[key]
      if (!mark) return name
      const label = labels[key] ?? name
      return `<span class="model-mention" title="${label} · 仅用于识别名称" data-provider="${label}"><img src="/assets/providers/${mark}" alt="" aria-hidden="true" draggable="false" width="14" height="14">${name}</span>`
    })).join('')
  }).join('')
}
