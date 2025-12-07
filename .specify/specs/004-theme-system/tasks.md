# Tasks: Theme System

**Input**: `spec.md` and `plan.md` under `.specify/specs/004-theme-system/`  
**Prerequisites**: None (theme system is independent).

## Phase 1: Theme Provider Setup (DONE)

- [x] T001 [US2] Install `next-themes` package
- [x] T002 [US2] Create `components/ThemeProvider.tsx` wrapper component
- [x] T003 [US2] Configure ThemeProvider with `defaultTheme="light"`
- [x] T004 [US2] Configure ThemeProvider with `enableSystem=false`
- [x] T005 [US2] Set `attribute="class"` for class-based dark mode
- [x] T006 [US2] Wrap app in `app/layout.tsx` with ThemeProvider
- [x] T007 [US2] Add `suppressHydrationWarning` to html tag in layout

## Phase 2: Theme Switcher Component (DONE)

- [x] T010 [US1] Create `components/ThemeSwitcher.tsx` component
- [x] T011 [US1] Import `useTheme` from next-themes
- [x] T012 [US1] Add mounted state to prevent hydration mismatch
- [x] T013 [US1] Use `resolvedTheme` to correctly detect current theme
- [x] T014 [US1] Implement toggleTheme function to switch between light/dark
- [x] T015 [US1] Add sun icon for light mode (when dark is active)
- [x] T016 [US1] Add moon icon for dark mode (when light is active)
- [x] T017 [US1] Show loading placeholder until component is mounted
- [x] T018 [US1] Add proper aria-label and title attributes

## Phase 3: Navbar Integration (DONE)

- [x] T020 [US1] Import ThemeSwitcher in `components/Navbar.tsx`
- [x] T021 [US1] Add ThemeSwitcher to navbar right side
- [x] T022 [US1] Style ThemeSwitcher to match navbar design
- [x] T023 [US1] Ensure ThemeSwitcher is visible in both themes

## Phase 4: Component Theme Support (DONE)

- [x] T030 [US3] Update `components/Navbar.tsx` with dark: variants
- [x] T031 [US3] Update `components/Sidebar.tsx` with dark: variants
- [x] T032 [US3] Update `components/Chatbot.tsx` with dark: variants
- [x] T033 [US3] Update `components/MarkdownContent.tsx` with dark: variants
- [x] T034 [US3] Update all page components with dark: variants
- [x] T035 [US3] Ensure proper contrast ratios in both themes

## Phase 5: Global CSS Theme Support (DONE)

- [x] T040 [US3] Update `app/globals.css` with CSS variables for themes
- [x] T041 [US3] Configure prose styles for light theme
- [x] T042 [US3] Configure prose styles for dark theme (dark:prose-invert)
- [x] T043 [US3] Update code block syntax highlighting for both themes
- [x] T044 [US3] Ensure link colors work in both themes
- [x] T045 [US3] Update table styles for both themes

## Phase 6: Testing & Verification (DONE)

- [x] T050 [US1] Test theme toggle (click button, verify theme changes)
- [x] T051 [US1] Test theme persistence (refresh page, verify theme maintained)
- [x] T052 [US3] Test all pages in light theme
- [x] T053 [US3] Test all pages in dark theme
- [x] T054 [US3] Verify no hydration mismatch errors in console
- [x] T055 [US3] Verify contrast ratios meet WCAG AA standards

## Completed Features Summary

✅ **Theme Toggle**: Instant theme switching via navbar button  
✅ **Theme Persistence**: User preference saved in localStorage  
✅ **Light Default**: Application defaults to light theme  
✅ **Consistent Application**: All components support both themes  
✅ **WCAG Compliance**: Contrast ratios meet accessibility standards  
✅ **Hydration Safe**: No hydration mismatches, smooth theme transitions

**All user stories (US1, US2, US3) are complete and independently testable.**

