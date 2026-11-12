# Skill loading reference

Reference: NousResearch/hermes-agent v0.23.1, commit
`3b421578588243f920886e6fe79a6faa87ea026e`,
[`tools/skills_tool.py`](https://github.com/NousResearch/hermes-agent/blob/3b421578588243f920886e6fe79a6faa87ea026e/tools/skills_tool.py),
especially `skills_list` and `skill_view`. Upstream MIT terms are retained in
`LICENSE.hermes.txt`.

LingTu implements the same progressive reading pattern in its existing Agent:
list names/descriptions, then read one skill or a referenced Markdown/text file.
This is a small local implementation, not the complete Hermes skill runtime.
The bundled task guides are written for LingTu's real tool contracts.

Local roots: packaged `decision/skills/` and
`$XDG_CONFIG_HOME/lingtu/skills/` (default `~/.config/lingtu/skills/`). IDs use
`builtin/` and `local/` prefixes so local additions cannot silently shadow bundled guides.
Read each call afresh; editing a guide does not require restarting the Agent.
Files are UTF-8, at most 64 KiB. References stay inside their skill directory.

No shell preprocessing, automatic dependency installation, skill downloading,
credential capture or executable scripts are activated by reading a skill.
These are meaningful differences from Hermes, whose skill_view can preprocess
content and ensure declared dependencies. Existing tool permissions and motion
ownership are unchanged. Runtime `@skill` discovery remains the owner of exposed
Python functions; these Markdown guides are a separate kind of content.
