# Data Model: Dark Mode and Visibility Fixes

## CSS Custom Properties (Variables)

### Dark Mode Color Variables
- `--ifm-color-content`: Text color that switches between light/dark modes
- `--ifm-color-background`: Background color that switches between light/dark modes
- `--ifm-color-gray-*`: Grayscale colors for consistent theming across modes
- `--ifm-color-emphasis-*`: Emphasis colors for text and backgrounds

### Layout Spacing Variables
- `--ifm-navbar-margin-bottom`: Spacing to prevent navbar overlap with content
- `--ifm-footer-padding`: Padding for proper footer spacing
- `--ifm-global-spacing`: Base spacing unit for consistent layout

## Component Structure

### Footer Component
- **Properties**:
  - background-color (theme-dependent)
  - color (theme-dependent)
  - padding
  - grid-template-columns (for column organization)
- **Relationships**: Child elements include footer links, copyright text, and sections

### Navbar Component
- **Properties**:
  - background-color (theme-dependent)
  - color (theme-dependent)
  - margin-bottom (to prevent content overlap)
  - logo visibility (hidden in this implementation)
- **Relationships**: Contains navigation links and header elements

### Skip Link Component
- **Properties**:
  - display: none (to remove visible element while maintaining accessibility)
  - position: absolute (for screen reader access)
- **Relationships**: Connected to main content area

### Content Text Elements
- **Properties**:
  - color (theme-dependent for contrast)
  - contrast ratio (WCAG AA compliant)
- **Relationships**: All text elements inherit theme-dependent colors

## Theme State Transitions

### Light to Dark Mode
- Background colors switch from light to dark
- Text colors switch from dark to light
- All elements maintain WCAG AA contrast ratios
- Footer elements update to dark background with light text

### Dark to Light Mode
- Background colors switch from dark to light
- Text colors switch from light to dark
- All elements maintain WCAG AA contrast ratios
- Footer elements update to light background with dark text

## Validation Rules

1. **Contrast Ratio**: All text must maintain ≥4.5:1 contrast ratio in both themes
2. **Visibility**: Navbar items must be clearly visible in both themes
3. **Spacing**: No overlap between navbar and main content
4. **Accessibility**: Skip link functionality maintained for screen readers
5. **Logo Removal**: Logo must be completely hidden from visual display
6. **Footer Organization**: Footer links must be organized in clear columns