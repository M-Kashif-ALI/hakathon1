---
id: 3
title: Fix website visibility plan
stage: plan
date: 2025-12-25
surface: agent
model: claude-sonnet-4-5-20250929
feature: fix-website-visibility
branch: 001-fix-website-visibility
user: Claude Code User
command: sp.plan
labels: ["planning", "architecture", "css", "docusaurus", "dark-mode"]
links:
  spec: specs/001-fix-website-visibility/spec.md
  plan: specs/001-fix-website-visibility/plan.md
  research: specs/001-fix-website-visibility/research.md
  data-model: specs/001-fix-website-visibility/data-model.md
  quickstart: specs/001-fix-website-visibility/quickstart.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-fix-website-visibility/plan.md
 - specs/001-fix-website-visibility/research.md
 - specs/001-fix-website-visibility/data-model.md
 - specs/001-fix-website-visibility/quickstart.md
 - .specify/memory/constitution.md
tests:
 - null
---

## Prompt

markdown/sp.plan Fix Physical AI & Humanoid Robotics Course website UI issues

## Architecture
- Docusaurus static site with CSS custom properties
- Config changes via `docusaurus.config.js`
- Styling via `src/css/custom.css`

## Implementation Phases

### Phase 1: Config Changes (30 min)
**File:** `docusaurus.config.js`
- Remove `navbar.logo` property
- Restructure `footer.links` into 3 columns:
  - Course Modules | Documentation | Community

### Phase 2: Base Fixes (30 min)
**File:** `src/css/custom.css`
- Hide skip link: `.skipToContent { position: absolute; left: -9999px; }`
- Add navbar spacing: `.navbar { margin-bottom: 2rem; }`
- Set light mode colors: `--ifm-color-content: #1c1e21;`

### Phase 3: Dark Mode (45 min)
**File:** `src/css/custom.css`
```css
[data-theme='dark'] {
  --ifm-color-content: #e3e3e3;
  --ifm-navbar-link-color: #ffffff;
}
[data-theme='dark'] .footer {
  background-color: #1c1e21;
  color: #e3e3e3;
}
```

### Phase 4: Validate (30 min)
- Test both modes on home/docs/chapter pages
- Check contrast ratios with DevTools
- Verify footer layout clean

## Key Decisions

**Skip Link:** Use `position: absolute; left: -9999px` (keeps accessibility, hides visually)

**Footer Columns:** 3 columns (balanced for current link count)

**Navbar Spacing:** `margin-bottom` on navbar (simpler than padding adjustments)

**Contrast Approach:** Override Docusaurus variables + component styles (thorough coverage)

## Testing Checklist
- [ ] Logo removed, navbar intact
- [ ] No skip link visible, no horizontal scroll
- [ ] Navbar separated from content
- [ ] Light mode: dark text, contrast ≥4.5:1
- [ ] Dark mode: light text, contrast ≥4.5:1, footer dark
- [ ] Footer in 3 clean columns
- [ ] Works on Chrome, Firefox, Safari

## Implementation Order
1. Remove logo + reorganize footer (config)
2. Hide skip link + navbar spacing (CSS)
3. Light mode text colors (CSS)
4. Dark mode overrides (CSS)
5. Test + adjust

**Total Time:** ~2.5 hours

## Response snapshot

Successfully created comprehensive implementation plan for fixing dark mode, visibility, and layout issues on the Physical AI & Humanoid Robotics Course website. The plan includes:

1. **Implementation Plan** (plan.md): Detailed technical approach with context, constitution check, and project structure
2. **Research Document** (research.md): Key decisions made with rationale and alternatives considered
3. **Data Model** (data-model.md): CSS custom properties, component structure, theme transitions, and validation rules
4. **Quickstart Guide** (quickstart.md): Implementation steps, testing checklist, and troubleshooting guide

The plan follows a CSS-only approach leveraging Docusaurus' existing theme system to:
- Fix footer color switching in dark mode
- Enhance text contrast for WCAG AA compliance
- Remove skip link visually while maintaining accessibility
- Completely hide the dinosaur logo
- Reorganize footer links into clear columns
- Ensure proper navbar spacing to prevent content overlap

All constitution requirements were validated, and the Claude agent context was updated with the new framework information. The implementation approach maintains all existing content structure while improving user experience and accessibility.

## Outcome

- ✅ Impact: Comprehensive planning completed for UI/UX fixes
- 🧪 Tests: Constitution check validated, all requirements met
- 📁 Files: Plan, research, data model, quickstart, and agent context updated
- 🔁 Next prompts: Ready for task generation and implementation
- 🧠 Reflection: CSS-only approach ensures maintainability and compatibility

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): All planning requirements met
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Generate implementation tasks