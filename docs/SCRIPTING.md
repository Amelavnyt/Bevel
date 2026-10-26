# Scripting

This is a stub. Bevel will embed the Rhai scripting language in a sandbox with no file or network access, operation and call-depth limits, and deterministic behaviour. Scripts are used in two ways. A macro runs once, and each call it makes becomes a normal Command, so the result lands in the timeline as ordinary features. A script feature is itself a timeline step that re-runs on every regeneration with declared parameters (a gear generator is the typical example), and it labels its outputs so topological names stay stable. See [ARCHITECTURE.md](ARCHITECTURE.md) and [ADR-0010](DECISIONS.md#adr-0010-rhai-for-scripting-script-features-via-an-injected-runner).

The API reference will be written during the "Scripting" phase (see [ROADMAP.md](ROADMAP.md)).
