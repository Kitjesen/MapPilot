import { createRoot } from 'react-dom/client'
import { Preview } from './RecordingPreview'
import { createRecordingPreviewFetch } from './fixtures/recordingPreview'
import '../src/App.css'

window.fetch = createRecordingPreviewFetch()
createRoot(document.getElementById('root')!).render(<Preview />)
