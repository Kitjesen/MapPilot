import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import { readFileSync } from 'node:fs'
import { fileURLToPath, URL } from 'node:url'

// Change ROBOT_HOST to your robot's IP for local development
const ROBOT_HOST = process.env.ROBOT_HOST || 'localhost:5050'
const APP_VERSION: string = JSON.parse(readFileSync(new URL('./package.json', import.meta.url), 'utf8')).version

export default defineConfig({
  plugins: [react()],
  define: {
    'import.meta.env.VITE_APP_VERSION': JSON.stringify(APP_VERSION),
  },
  server: {
    port: 3000,
    fs: {
      // Guide pages import the repository's Markdown source at build time.
      allow: [fileURLToPath(new URL('..', import.meta.url))],
    },
    proxy: {
      '/api': `http://${ROBOT_HOST}`,
      '/ws': { target: `ws://${ROBOT_HOST}`, ws: true },
      '/mcp': `http://${ROBOT_HOST}`,
      '/map': `http://${ROBOT_HOST}`,
    },
  },
  build: {
    outDir: 'dist',
    emptyOutDir: true,
    rollupOptions: {
      input: {
        app: fileURLToPath(new URL('./index.html', import.meta.url)),
        guide: fileURLToPath(new URL('./guide/index.html', import.meta.url)),
      },
    },
  },
})
