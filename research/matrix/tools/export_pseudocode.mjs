// Export REA-returned text without rewriting the raw tool response.
import fs from 'node:fs/promises';
import path from 'node:path';

const [resultPath, outputDirectory] = process.argv.slice(2);
if (!resultPath || !outputDirectory) throw new Error('Usage: node export_pseudocode.mjs RESULT.json NEW_OUTPUT_DIR');
const result = JSON.parse(await fs.readFile(resultPath, 'utf8'));
const evidence = result.structuredContent;
const items = evidence.normalized_result.items;
await fs.mkdir(outputDirectory, {recursive: false});
const functions = [];
for (const item of items) {
  if (item.status !== 'ok' || typeof item.pseudocode !== 'string')
    throw new Error(`Incomplete decompilation: ${item.address}`);
  const filename = `decompile-${item.address}.txt`;
  await fs.writeFile(path.join(outputDirectory, filename), item.pseudocode, {flag: 'wx'});
  functions.push({address: item.address, procedure: item.procedure, status: item.status,
                 file: filename, characters: item.pseudocode.length, limitations: item.limitations ?? null});
}
await fs.writeFile(path.join(outputDirectory, 'index.json'), JSON.stringify({
  evidence_id: evidence.evidence_id, subject: evidence.subject,
  analysis_profile: evidence.analysis_profile, confidence: evidence.confidence,
  limitations: evidence.limitations, functions,
}, null, 2)+'\n', {flag: 'wx'});
console.log(JSON.stringify({evidence_id: evidence.evidence_id, functions: functions.length,
                           statuses: functions.map(item => item.status)}));
