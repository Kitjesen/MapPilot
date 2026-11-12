# Web search and extraction provenance

Adapted from NousResearch/hermes-agent v0.23.1, commit
`3b421578588243f920886e6fe79a6faa87ea026e`,
[`plugins/web/tavily/provider.py`](https://github.com/NousResearch/hermes-agent/blob/3b421578588243f920886e6fe79a6faa87ea026e/plugins/web/tavily/provider.py).
Copyright (c) 2025 Nous Research. MIT license: [LICENSE.hermes.txt](LICENSE.hermes.txt).

LingTu retains the `/search` and `/extract` payloads, Bearer authentication and
source URL/content normalization. Uses existing asynchronous httpx so cancelling
a conversation cancels its network request. Bounded results feed the existing
Agent tool loop, history, timings and UI. No Hermes process or plugin runtime.

Local differences: fixed Tavily API endpoint; server-side `TAVILY_API_KEY` is
required; no implicit keyless fallback, provider rotation, SDK or extra package.
Five search results, one extracted page, 30-second network timeout; snippets are
limited to 1200 characters and extracted content to 16000, with explicit truncation.
Provider error bodies are not forwarded to conversation history. Returned URLs
must be HTTP(S) without credentials; remote text is reference data, not instructions.
