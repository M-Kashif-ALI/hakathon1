# Implementation Plan: Fix Dark Mode, Visibility, and Layout Issues on Physical AI & Humanoid Robotics Course Website

**Branch**: `001-fix-website-visibility` | **Date**: 2025-12-25 | **Spec**: [link](./spec.md)
**Input**: Feature specification from `/specs/001-fix-website-visibility/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of dark mode, visibility, footer layout, navbar overlap, and spacing fixes for the Physical AI & Humanoid Robotics Course website. The primary requirement is to enhance the user experience by ensuring proper dark mode functionality, improving text contrast to meet WCAG AA standards, removing problematic elements like the dinosaur logo, and organizing the footer content in a clean, accessible manner. The approach involves CSS custom properties overrides and configuration changes in the Docusaurus framework.

## Technical Context

**Language/Version**: TypeScript/JavaScript (Docusaurus v3.9.2)
**Primary Dependencies**: Docusaurus framework, React, Infima CSS framework
**Storage**: N/A (static site)
**Testing**: Manual testing with browser dev tools for contrast ratios and layout validation
**Target Platform**: Web (static site deployed to GitHub Pages)
**Project Type**: Single web project (Docusaurus documentation site)
**Performance Goals**: Maintain existing performance scores, ensure CSS changes don't regress Lighthouse scores
**Constraints**: Must preserve all existing content structure and routing, maintain WCAG AA accessibility compliance, ensure cross-browser compatibility
**Scale/Scope**: Single documentation website with multiple pages and sections

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

1. **Spec-first development**: All implementation must be directly generated from and aligned with Spec-Kit Plus specs ✓
2. **AI-native authoring**: All code generation follows AI-assisted development patterns with Claude Code ✓
3. **Zero hallucinations**: No invented data, APIs, or contracts; strict adherence to existing codebase and documentation ✓
4. **Developer-focused clarity**: Code examples must be correct and runnable; documentation must be clear and reproducible ✓
5. **RAG Grounding Constraint**: RAG chatbot answers must be grounded only in indexed book content; no answers outside indexed content ✓
6. **Performance and Efficiency**: Low-latency responses required; efficient indexing and retrieval mechanisms ✓

## Project Structure

### Documentation (this feature)

```text
specs/001-fix-website-visibility/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
my-book/
├── docusaurus.config.ts     # Docusaurus configuration
├── sidebars.ts              # Navigation structure
├── src/
│   ├── components/          # Custom React components
│   │   └── Breadcrumb.js    # Breadcrumb component
│   ├── css/
│   │   └── custom.css       # Custom CSS overrides
│   └── pages/               # Static pages
├── docs/                    # Documentation content
├── static/                  # Static assets
└── package.json             # Project dependencies
```

**Structure Decision**: Single Docusaurus documentation project with CSS overrides in custom.css and configuration changes in docusaurus.config.ts. This structure was selected because the changes are primarily UI/UX focused and can be implemented through CSS custom properties and Docusaurus configuration options without requiring additional backend services or complex architecture.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | N/A | N/A |
