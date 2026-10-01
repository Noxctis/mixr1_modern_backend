# Commit Message Instructions

Generate commit messages following the Conventional Commits 1.0.0 specification.
This repository is part of an academic thesis, so every commit must be
self-explanatory, traceable, and useful as a development record.

## Format

<type>(<scope>): <subject>

<body>

<footer>

## Subject line (required)
- Maximum 50 characters; never exceed 72.
- Imperative, present tense: "add", "fix", "refactor" (not "added" or "adds").
- Lowercase after the colon, no trailing period.
- Describe WHAT changed at a high level, specific enough to be understood
  without opening the diff. Avoid vague words like "update", "changes", "stuff", "fix bug".

## Types
- feat: new feature or capability
- fix: bug fix
- docs: documentation or thesis-writing changes only
- style: formatting, whitespace, no logic change
- refactor: code restructuring with no behavior change
- perf: performance improvement
- test: adding or correcting tests
- build: build system, dependencies, packaging
- ci: CI/CD configuration
- chore: maintenance tasks, tooling, config
- revert: reverts a previous commit (reference the reverted hash)
- data: dataset changes, preprocessing, schema changes (if applicable)
- exp: experiments, model tuning, benchmarks (if applicable)

## Scope (strongly encouraged)
- Short noun for the affected module, component, or area, e.g. auth, api,
  database, ui, model, dataset, eval, docs, thesis.
- Use the same scope names consistently across commits.
- Omit only if the change spans the entire project.

## Body (required for any non-trivial change)
- Leave one blank line after the subject.
- Wrap lines at 72 characters.
- Explain WHAT changed and WHY, not line-by-line HOW.
- Include, where relevant:
  - Motivation: the problem or requirement that prompted the change
  - Approach: key design decision and alternatives considered or rejected
  - Impact: affected components, behavior changes, performance or accuracy effects
  - Results: for experiments, include metrics, parameters, or findings
  - Limitations or follow-up work still needed
- Use bullet points ("- ") for multiple distinct changes.
- Skip the body only for truly trivial commits (typo, formatting).

## Footer
- Reference issues and tasks: `Refs: #12`, `Closes: #34`, `Fixes: #56`.
- Breaking changes: start a footer line with `BREAKING CHANGE: <description>`
  and add `!` after the type/scope in the subject, e.g. `feat(api)!: ...`.
- Thesis traceability (when applicable):
  - `Thesis-Ref: Chapter 3, Section 3.2`
  - `Experiment: EXP-014`
- Co-authors: `Co-authored-by: Name <email>`.

## Rules
- One logical change per commit. If the diff mixes unrelated changes,
  describe the primary one and mention the rest in the body.
- Never mention "this commit" or "this PR" in the message.
- Do not include file names unless essential for clarity.
- No emojis, no filler, no generic messages.
- Write in English.

## Example

feat(auth): add JWT refresh token rotation

Access tokens previously expired after 24 hours with no renewal path,
forcing users to log in again and breaking long-running sessions.

- Add refresh token endpoint with single-use rotation
- Store hashed refresh tokens in the sessions table
- Invalidate the entire token family on reuse detection

Chose rotation over fixed-lifetime refresh tokens to limit the damage
of a leaked token. Token lifetime is configurable via REFRESH_TTL.

Closes: #42
Thesis-Ref: Chapter 4, Section 4.3