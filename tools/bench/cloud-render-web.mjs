import { mkdir, writeFile } from 'node:fs/promises'
import { fileURLToPath, pathToFileURL } from 'node:url'
import path from 'node:path'

const repo = fileURLToPath(new URL('../../', import.meta.url))
const web = path.join(repo, 'web')
const output = path.join(repo, 'artifacts/web/cloud-performance-20261110/result.json')
const { createServer } = await import(pathToFileURL(path.join(web, 'node_modules/vite/dist/node/index.js')).href)
const server = await createServer({
  configFile: false, root: web, appType: 'mpa',
  server: { host: '127.0.0.1', port: 5190, strictPort: true, fs: { allow: [repo] },
    proxy: { '/api': 'http://127.0.0.1:4326' } },
  plugins: [{ name: 'cloud-benchmark-result', configureServer(vite) {
    vite.middlewares.use('/__bench-result', async (request, response, next) => {
      if (request.method !== 'POST') return next()
      try {
        const chunks = []
        for await (const chunk of request) chunks.push(chunk)
        const result = JSON.parse(Buffer.concat(chunks).toString('utf8'))
        if (!result || typeof result !== 'object' || result.map !== 'test6_cleaned') throw new Error('Invalid benchmark result')
        await mkdir(path.dirname(output), { recursive: true })
        await writeFile(output, JSON.stringify(result, null, 2) + '\n')
        response.writeHead(200, { 'Content-Type': 'application/json' })
        response.end('{"saved":true}')
        process.stdout.write(`Saved ${output}\n`)
      } catch (error) {
        response.writeHead(400, { 'Content-Type': 'text/plain' })
        response.end(String(error))
      }
    })
  } }],
})
await server.listen()
process.stdout.write('http://127.0.0.1:5190/tests/cloud-render-bench.html\n')
for (const signal of ['SIGINT', 'SIGTERM']) process.on(signal, async () => { await server.close(); process.exit(0) })
