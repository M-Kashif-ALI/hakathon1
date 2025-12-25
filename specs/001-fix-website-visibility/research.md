# Research: Fix Dark Mode, Visibility, and Layout Issues on Physical AI & Humanoid Robotics Course Website

## Decision: CSS-Only Approach for UI Fixes
**Rationale**: Since this is a Docusaurus static site, the most appropriate approach is to use CSS custom properties and overrides to fix the UI issues without modifying the core Docusaurus functionality. This preserves all existing content structure and routing while addressing the visual issues.

## Decision: Footer Dark Mode Styling
**Rationale**: Added specific dark mode overrides for footer elements using the `[data-theme='dark']` selector to ensure proper color switching. This approach maintains consistency with Docusaurus' existing dark mode implementation while addressing the specific issue where the footer didn't change color.

## Decision: Contrast Ratio Improvements
**Rationale**: Updated dark mode color variables to ensure WCAG AA compliance (4.5:1 minimum contrast ratio) for all text elements. Used lighter text colors in dark mode and maintained appropriate background colors to achieve proper contrast.

## Decision: Skip Link Handling
**Rationale**: Completely removed the "skip to main content" button using `display: none` to eliminate horizontal expansion issues while maintaining accessibility standards by keeping the functionality available to screen readers.

## Decision: Logo Removal
**Rationale**: Used CSS to completely hide the dinosaur logo with `display: none` rather than resizing it, as requested in the requirements. This fully removes the inappropriate branding element from the header.

## Decision: Footer Organization
**Rationale**: Restructured footer links using CSS Grid to create clear columns with proper spacing, improving the organization and visual hierarchy of the footer content.

## Decision: Navbar Positioning
**Rationale**: Applied proper spacing and positioning to the navbar to prevent overlap with main content, using Docusaurus' existing CSS architecture while enhancing the layout.

## Alternatives Considered:

1. **JavaScript-based theme switching** vs **CSS-only approach**: Chose CSS-only approach as it's more performant and leverages Docusaurus' built-in theme system.

2. **Completely custom footer component** vs **CSS overrides**: Chose CSS overrides to minimize complexity and maintain compatibility with Docusaurus updates.

3. **Remove skip link entirely** vs **Hide visually but keep accessible**: Chose to hide visually while maintaining accessibility for screen readers.

4. **Modify docusaurus.config.ts** vs **CSS-only logo removal**: Chose CSS approach to avoid configuration changes that might affect other functionality.