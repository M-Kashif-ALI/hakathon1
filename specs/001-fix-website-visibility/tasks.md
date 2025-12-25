# Implementation Tasks: Fix Dark Mode, Visibility, and Layout Issues on Physical AI & Humanoid Robotics Course Website

**Feature**: Fix Dark Mode, Visibility, and Layout Issues on Physical AI & Humanoid Robotics Course Website
**Branch**: `001-fix-website-visibility`
**Created**: 2025-12-25
**Input**: Feature specification from `/specs/001-fix-website-visibility/spec.md`

## Summary

Implementation of dark mode, visibility, footer layout, navbar overlap, and spacing fixes for the Physical AI & Humanoid Robotics Course website. The approach will use CSS custom properties and overrides to address all UI/UX issues while preserving existing content structure and functionality. The implementation will follow a phased approach with foundational work first, followed by user story implementations in priority order.

## Implementation Strategy

- **MVP First**: Focus on User Story 1 (Enhanced Dark Mode Experience) as the minimum viable product
- **Incremental Delivery**: Each user story builds on the previous, creating independently testable increments
- **Performance Priority**: Maintain or improve Lighthouse scores throughout implementation
- **Accessibility First**: Ensure WCAG 2.1 AA compliance from the start

## Dependencies

- **User Story 2** depends on foundational dark mode styling from **User Story 1**
- **User Story 3** depends on foundational CSS variables from **User Story 1**

## Parallel Execution Examples

- **Tasks T005-T008** (CSS variables, footer styling, navbar improvements) can be executed in parallel
- **Tasks T015-T020** (skip link, logo removal, footer organization) can be executed in parallel after foundational work
- **Tasks T025-T030** (layout fixes, spacing adjustments) can be executed in parallel after foundational work

## Phase 1: Setup

- [X] T001 Create backup of current custom.css file before making changes
- [X] T002 Establish baseline Lighthouse performance scores for current site
- [X] T003 Document current color palette and contrast ratios for reference
- [X] T004 Set up development environment with yarn start for real-time testing

## Phase 2: Foundational Design System

- [X] T005 [P] Define and implement new dark mode color system in CSS variables in my-book/src/css/custom.css
- [X] T006 [P] Implement WCAG 2.1 AA compliant contrast ratios for all text elements in my-book/src/css/custom.css
- [X] T007 [P] Establish spacing system with consistent units in my-book/src/css/custom.css
- [X] T008 [P] Create CSS variable definitions for dark mode color variants in my-book/src/css/custom.css
- [X] T009 [P] Set up CSS custom properties for consistent theming across components in my-book/src/css/custom.css
- [X] T010 Test foundational color system implementation with sample pages

## Phase 3: User Story 1 - Enhanced Dark Mode Experience (Priority: P1)

**Goal**: When users access the Physical AI & Humanoid Robotics Course website, they should experience proper dark mode functionality where all elements correctly switch between light and dark themes.

**Independent Test**: Can verify that all elements properly switch between light and dark themes, with appropriate contrast ratios (4.5:1 minimum) maintained in both modes, and that the visual experience is consistent across all page sections.

- [X] T011 [US1] [P] Update footer dark mode styling to have dark background with light text in my-book/src/css/custom.css
- [X] T012 [US1] [P] Implement proper dark mode text contrast for content areas in my-book/src/css/custom.css
- [X] T013 [US1] [P] Enhance navbar styling for better visibility in dark mode in my-book/src/css/custom.css
- [X] T014 [US1] [P] Improve dark mode contrast for all text elements in my-book/src/css/custom.css
- [X] T015 [US1] Test dark mode color switching functionality across all page sections
- [X] T016 [US1] Verify WCAG AA contrast ratios meet 4.5:1 minimum in dark mode
- [X] T017 [US1] Test dark mode functionality across different browsers and devices

## Phase 4: User Story 2 - Clean and Organized Layout (Priority: P1)

**Goal**: When users navigate the website, they should experience a clean, well-organized layout without visual clutter, overlapping elements, or spacing issues.

**Independent Test**: Can verify that all layout elements are properly positioned without overlap, spacing is consistent, and content is organized in a logical, accessible manner.

- [X] T018 [US2] [P] Fix navbar positioning to prevent overlap with main content in my-book/src/css/custom.css
- [X] T019 [US2] [P] Organize footer links into clear columns with proper spacing in my-book/src/css/custom.css
- [X] T020 [US2] [P] Implement consistent spacing system across all page sections in my-book/src/css/custom.css
- [X] T021 [US2] [P] Ensure proper spacing between navbar and content area in my-book/src/css/custom.css
- [X] T022 [US2] Test layout fixes across different screen sizes and devices
- [X] T023 [US2] Verify no overlapping elements on any page sections
- [X] T024 [US2] Validate responsive behavior across breakpoints (320px, 768px, 1024px, 1440px)

## Phase 5: User Story 3 - Accessibility and Branding Improvements (Priority: P2)

**Goal**: When users with accessibility needs access the site, they should encounter properly implemented accessibility features and appropriate branding elements.

**Independent Test**: Can verify that accessibility features work properly without causing layout issues, and that branding elements enhance rather than detract from the user experience.

- [X] T025 [US3] [P] Remove "skip to main content" button visually while maintaining accessibility in my-book/src/css/custom.css
- [X] T026 [US3] [P] Completely hide dinosaur logo from header area in my-book/src/css/custom.css
- [X] T027 [US3] [P] Ensure skip link remains accessible to screen readers in my-book/src/css/custom.css
- [X] T028 [US3] [P] Test accessibility features with screen readers and keyboard navigation
- [X] T029 [US3] Verify logo removal doesn't affect navbar layout or spacing
- [X] T030 [US3] Test accessibility improvements across different browsers

## Phase 6: Polish & Cross-Cutting Concerns

- [X] T031 Implement accessibility enhancements (focus states, ARIA attributes) in my-book/src/css/custom.css
- [X] T032 Optimize performance and bundle size after all CSS changes are implemented
- [X] T033 Run comprehensive Lighthouse audit to verify performance scores remain high
- [X] T034 delete the skip to main content button entirely from the website
- [X] T035 Test cross-browser compatibility (Chrome, Firefox, Safari, Edge)
- [X] T036 Verify all existing functionality remains intact after UI/UX changes
- [X] T037 Update documentation in README or contributing guides with new design standards
- [X] T038 Perform final accessibility audit using automated tools
- [X] T039 Test site functionality with keyboard navigation only
- [X] T040 Verify all links and navigation elements work correctly
- [X] T041 Final review and polish of all UI/UX improvements

## MVP Scope (User Story 1 Complete)

The minimum viable product includes tasks T001-T017, which deliver the enhanced dark mode experience with proper color switching, WCAG AA contrast ratios, and consistent visual experience across both themes while maintaining all existing functionality.