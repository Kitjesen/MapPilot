import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'

import { DocsApp } from './DocsApp'
import './docs.css'

const root = document.getElementById('root')

if (!root) {
  throw new Error('LingTu Docs root element is missing.')
}

createRoot(root).render(
  <StrictMode>
    <DocsApp />
  </StrictMode>,
)
