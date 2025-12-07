<!-- Sync Impact Report -->
<!--
Version change: 0.0.0 -> 1.0.0
Modified principles: None (initial population)
Added sections: None (initial population)
Removed sections: None (initial population)
Templates requiring updates:
- .specify/templates/plan-template.md: ✅ updated
- .specify/templates/spec-template.md: ✅ updated
- .specify/templates/tasks-template.md: ✅ updated
- .specify/templates/commands/sp.constitution.md: ✅ updated
Follow-up TODOs: None
-->
# Spec-Kit Constitution

## Core Principles

### I. Library-First
Every feature starts as a standalone library. Libraries MUST be self-contained, independently testable, and documented. A clear purpose is required - no organizational-only libraries.

### II. CLI Interface
Every library exposes functionality via CLI. Text in/out protocol: stdin/args → stdout, errors → stderr. Support JSON + human-readable formats.

### III. Test-First (NON-NEGOTIABLE)
TDD is mandatory: Tests MUST be written → User approved → Tests fail → Then implement. The Red-Green-Refactor cycle is strictly enforced.

### IV. Integration Testing
Focus areas requiring integration tests: New library contract tests, Contract changes, Inter-service communication, Shared schemas.

### V. Observability, Versioning & Breaking Changes, Simplicity
Text I/O ensures debuggability; Structured logging is required. MAJOR.MINOR.PATCH format for versioning; Breaking changes require explicit justification and communication. Start simple, adhere to YAGNI principles.

### VI. Small, Testable Changes
All changes MUST be small, testable, and reference code precisely. Prefer the smallest viable diff; do not refactor unrelated code.

## Additional Constraints

Technology stack requirements, compliance standards, deployment policies.

## Development Workflow

Code review requirements, testing gates, deployment approval process.

## Governance

Constitution supersedes all other practices. Amendments require documentation, approval, and a migration plan. All PRs/reviews MUST verify compliance. Complexity MUST be justified.

**Version**: 1.0.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-04