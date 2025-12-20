---
id: 1
title: Update project constitution for AI-Spec Book
stage: constitution
date: 2025-12-20
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: main
user: user
command: /sp.constitution
labels: ["constitution", "spec-first", "ai-development", "rag-chatbot"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - .specify/templates/plan-template.md
  - .specify/templates/spec-template.md
  - .specify/templates/tasks-template.md
tests: []
---

## Prompt

```
/sp.constitution

Project: AI-Spec–Driven Book with Embedded RAG Chatbot

Core principles:
- Spec-first development (Spec-Kit Plus as source of truth)
- AI-native authoring (Claude Code)
- Accuracy, traceability, and zero hallucinations
- Developer-focused clarity and reproducibility

Key standards:
- Book content generated strictly from Spec-Kit Plus specs
- Written as Docusaurus docs and deployed to GitHub Pages
- Code examples must be correct and runnable
- RAG chatbot answers grounded only in indexed book content
- Support full-book QA and selected-text–only QA
- Responses must reference document sections

Stack constraints:
- Specs: Spec-Kit Plus
- Authoring: Claude Code
- Docs: Docusaurus
- Backend: FastAPI
- LLM orchestration: OpenAI Agents / ChatKit SDKs
- Vector DB: Qdrant Cloud (Free Tier)
- DB: Neon Serverless Postgres
- Deployment: GitHub Pages

RAG constraints:
- No answers outside indexed content
- Strict context isolation for selected-text mode
- Fallback when answer not found
- Low-latency
```

## Response snapshot

Constitution successfully created with version 1.0.0 containing 6 core principles:
1. Spec-first development
2. AI-native authoring
3. Zero hallucinations
4. Developer-focused clarity
5. RAG Grounding Constraint
6. Performance and Efficiency

Templates updated to align with constitution:
- plan-template.md: Added constitution checks section
- spec-template.md: Added constitution alignment section
- tasks-template.md: Added constitution compliance tasks and strategy

## Outcome

- ✅ Impact: Initial project constitution established with core principles
- 🧪 Tests: No tests required for constitution update
- 📁 Files: 4 files updated to maintain consistency
- 🔁 Next prompts: Proceed with feature specification using new constitution
- 🧠 Reflection: Constitution provides clear governance framework for AI-assisted development

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin feature specification following constitution principles