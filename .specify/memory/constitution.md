<!-- SYNC IMPACT REPORT:
Version change: undefined -> 1.0.0
Modified principles: none (new constitution)
Added sections: All principles and sections (new constitution)
Removed sections: none
Templates requiring updates: ✅ updated - .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md
Follow-up TODOs: none
-->
# AI-Spec–Driven Book with Embedded RAG Chatbot Constitution

## Core Principles

### Spec-first development
All development starts from Spec-Kit Plus specifications as the single source of truth; No feature development without corresponding spec; All code and documentation must be directly generated from and aligned with specs

### AI-native authoring
Claude Code serves as the primary authoring agent; All code generation and documentation follows AI-assisted development patterns; Maintain accuracy and traceability throughout AI-human collaboration

### Zero hallucinations
All outputs must be factually accurate and grounded in available information; No invented data, APIs, or contracts; Strict adherence to existing codebase and documentation

### Developer-focused clarity
Code examples must be correct and runnable; Documentation must be clear and reproducible; Focus on developer experience and usability

### RAG Grounding Constraint
RAG chatbot answers must be grounded only in indexed book content; No answers outside indexed content; Strict context isolation for selected-text mode with proper fallback mechanisms

### Performance and Efficiency
Low-latency responses required; Efficient indexing and retrieval mechanisms; Optimize for speed and resource usage

## Stack and Technology Constraints
Specs: Spec-Kit Plus; Authoring: Claude Code; Docs: Docusaurus; Backend: FastAPI; LLM orchestration: OpenAI Agents / ChatKit SDKs; Vector DB: Qdrant Cloud (Free Tier); DB: Neon Serverless Postgres; Deployment: GitHub Pages

## Development Workflow
Book content generated strictly from Spec-Kit Plus specs; Written as Docusaurus docs and deployed to GitHub Pages; Code examples must be correct and runnable; Support full-book QA and selected-text–only QA; Responses must reference document sections

## Governance
Constitution supersedes all other practices; Amendments require documentation and approval; All development must comply with Spec-first principles; Complexity must be justified with clear benefits

**Version**: 1.0.0 | **Ratified**: 2025-12-20 | **Last Amended**: 2025-12-20