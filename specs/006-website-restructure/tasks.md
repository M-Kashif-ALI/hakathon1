---
description: "Task list for website restructure and enhancement"
---

# Tasks: Website Restructure and Enhancement

**Input**: Design documents from `/specs/006-website-restructure/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Configuration**: `my-book/docusaurus.config.ts` for site configuration
- **Landing Page**: `my-book/src/pages/index.tsx` for landing page content
- **Documentation**: `my-book/docs/` for documentation structure
- **Internationalization**: `my-book/i18n/` for language files
- **Animations**: CSS in `my-book/src/css/custom.css` or component styles

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create backup of current docusaurus.config.ts file
- [ ] T002 [P] Create backup of current landing page at my-book/src/pages/index.tsx
- [ ] T003 [P] Create backup of current sidebar configuration
- [ ] T004 Set up i18n directory structure at my-book/i18n/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T005 Update site title and tagline in my-book/docusaurus.config.ts
- [ ] T006 [P] Configure i18n plugin in my-book/docusaurus.config.ts for English, Urdu, French, Spanish
- [ ] T007 [P] Create language directory structure at my-book/i18n/{en,ur,fr,es}/
- [ ] T008 Create basic translation files for all languages at my-book/i18n/*/docusaurus-theme-classic/

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Access Updated Website (Priority: P1) 🎯 MVP

**Goal**: Update website name to "Physical AI & Humanoid Robotics Course" with new content and animations

**Independent Test**: Visit the website and verify the new name appears in title, navigation, and content with animations

### Implementation for User Story 1

- [ ] T009 [US1] Update site metadata and title in my-book/docusaurus.config.ts
- [ ] T010 [P] [US1] Create new landing page content in my-book/src/pages/index.tsx
- [ ] T011 [P] [US1] Add CSS animations to landing page in my-book/src/css/custom.css
- [ ] T012 [US1] Update navigation links to reflect new course name
- [ ] T013 [US1] Test landing page animations for performance and accessibility
- [ ] T014 [US1] Verify new course name appears consistently throughout site

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Use Language Translation (Priority: P1)

**Goal**: Implement language translation functionality for English, Urdu, French, and Spanish

**Independent Test**: Use the translation interface to switch between available languages

### Implementation for User Story 2

- [ ] T015 [US2] Create English translation files at my-book/i18n/en/
- [ ] T016 [P] [US2] Create Urdu translation files at my-book/i18n/ur/
- [ ] T017 [P] [US2] Create French translation files at my-book/i18n/fr/
- [ ] T018 [P] [US2] Create Spanish translation files at my-book/i18n/es/
- [ ] T019 [US2] Configure language switcher component in my-book/docusaurus.config.ts
- [ ] T020 [US2] Test language switching functionality across all pages
- [ ] T021 [US2] Verify translation fallback handling for missing translations

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Navigate Module Structure (Priority: P2)

**Goal**: Restructure Module 1 to include ROS 2 Robotic Nervous System with three specific chapters

**Independent Test**: Navigate through the first module and verify all three chapters are properly organized

### Implementation for User Story 3

- [ ] T022 [US3] Create new module directory structure at my-book/docs/001-ros2-nervous-system/
- [ ] T023 [P] [US3] Move ros2-architecture content to my-book/docs/001-ros2-nervous-system/ros2-architecture/
- [ ] T024 [P] [US3] Move python-robot-control content to my-book/docs/001-ros2-nervous-system/python-robot-control/
- [ ] T025 [P] [US3] Move urdf-humanoid-structure content to my-book/docs/001-ros2-nervous-system/urdf-humanoid-structure/
- [ ] T026 [US3] Update sidebars.ts to reflect new module structure
- [ ] T027 [US3] Update all internal links to point to new locations
- [ ] T028 [US3] Test navigation and links within restructured module

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T029 [P] Test multi-language functionality with restructured modules
- [ ] T030 [P] Verify SEO-friendly URLs work across all languages
- [ ] T031 Update module introduction with learning objectives
- [ ] T032 Add cross-module references and navigation links
- [ ] T033 Test complete user journey in all supported languages
- [ ] T034 Validate all animations work across different languages and modules
- [ ] T035 Run accessibility checks on new content and features
- [ ] T036 Update documentation with new structure information

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All content files within a user story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all landing page updates for User Story 1 together:
Task: "Create new landing page content in my-book/src/pages/index.tsx"
Task: "Add CSS animations to landing page in my-book/src/css/custom.css"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test updated website name and basic functionality
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

### Constitution Compliance Strategy

- **Spec-first development**: All implementation directly from Spec-Kit Plus specs
- **Zero hallucinations**: No invented data, APIs, or contracts beyond spec
- **Developer-focused clarity**: All documentation correct and educational
- **Performance and Efficiency**: Content optimized for documentation loading

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence