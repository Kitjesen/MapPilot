export type AnswerSourceLike = { name: string; title: string }

export function answerCopyText(answer: string, sources: readonly AnswerSourceLike[]) {
  const text = answer.replace(/\*\*(.*?)\*\*/g, '$1').replace(/\[(\d+(?:,\s*\d+)*)\]/g, '')
  const list = sources.map((source, index) => `${index + 1}. ${source.name}: ${source.title}`).join('\n')
  return `${text.trim()}${list ? `\n\nSources:\n${list}` : ''}`
}

export function answerBlocks(answer: string) {
  return answer.split(/\n\s*\n/).filter(block => block.trim())
}
