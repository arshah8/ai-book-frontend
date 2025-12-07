# Tasks: Physical AI Book & Documentation UI

**Input**: Design documents from `/specs/001-physical-ai-docs-ui/`
**Prerequisites**: plan.md ✅, spec.md ✅

**Status**: All tasks completed ✅

## Phase 1: Setup & Infrastructure ✅

- [x] T001 Create Next.js App Router structure (`app/`, `components/`, `lib/`)
- [x] T002 Install dependencies: `react-markdown`, `remark-gfm`, `rehype-highlight`, `next-themes`
- [x] T003 Configure Tailwind CSS with custom typography and theme variables
- [x] T004 Create `lib/markdown-loader.ts` utility for loading markdown files

## Phase 2: Markdown Rendering ✅

- [x] T005 [P] Create `components/MarkdownContent.tsx` with syntax highlighting
- [x] T006 [P] Configure code block styling for light/dark themes
- [x] T007 [P] Add prose typography classes for headings, paragraphs, lists, links

## Phase 3: Navigation & Sidebar ✅

- [x] T008 [P] Create `components/Sidebar.tsx` with module/section structure
- [x] T009 [P] Implement collapsible sections and active route highlighting
- [x] T010 [P] Create `components/MobileSidebarToggle.tsx` with fixed positioning
- [x] T011 [P] Build `components/DocsLayout.tsx` wrapper component
- [x] T012 [US1] Integrate sidebar into all module pages (`/intro`, `/module1-4`, `/capstone`)
- [x] T013 [US2] Ensure sidebar closes on mobile after navigation
- [x] T014 [US2] Fix horizontal scrolling issues on mobile

## Phase 4: Theming System ✅

- [x] T015 [P] Install and configure `next-themes` package
- [x] T016 [P] Create `components/ThemeProvider.tsx` with light theme default
- [x] T017 [P] Build `components/ThemeSwitcher.tsx` with sun/moon icons
- [x] T018 [US1] Integrate theme provider in root layout
- [x] T019 [US1] Style all components for light and dark themes
- [x] T020 [FR-007] Verify WCAG AA contrast in both themes

## Phase 5: Dynamic Content Loading ✅

- [x] T021 [US1] Create `app/intro/page.tsx` loading `book/docs/intro.md`
- [x] T022 [US1] Create `app/module1/page.tsx` with dynamic TOC generation
- [x] T023 [US1] Create `app/module1/[section]/page.tsx` for dynamic sections
- [x] T024 [US1] Replicate module structure for modules 2, 3, 4, and capstone
- [x] T025 [US1] Add "Previous/Next" navigation links between sections

## Phase 6: Chatbot Integration ✅

- [x] T026 [US3] Create `components/Chatbot.tsx` with floating button
- [x] T027 [US3] Implement chat modal with authentication check
- [x] T028 [US3] Add selected text capture via `selectionchange` event
- [x] T029 [US3] Preserve selected text while typing (fix disappearing issue)
- [x] T030 [US3] Connect to backend `/api/chat` endpoint
- [x] T031 [US3] Make chatbot mobile-responsive (fixed button, modal sizing)

## Phase 7: Home Page & Polish ✅

- [x] T032 [P] Update `app/page.tsx` with module cards (remove duplicate Sign In/Up)
- [x] T033 [P] Style module cards with improved light theme colors
- [x] T034 [P] Add "Start Learning" CTA button
- [x] T035 [P] Ensure navbar shows Sign In/Up links (removed from hero)

## Phase 8: Mobile Optimization ✅

- [x] T036 [US2] Fix sidebar toggle button positioning (moved to right side)
- [x] T037 [US2] Ensure sidebar and chat button remain fixed while scrolling
- [x] T038 [US2] Prevent horizontal scrolling after sidebar navigation
- [x] T039 [US2] Test on multiple mobile viewport sizes

## Phase 9: Testing & Verification ✅

- [x] T040 [SC-001] Verify all routes render without errors
- [x] T041 [SC-002] Test mobile viewports for horizontal scroll issues
- [x] T042 [SC-003] Test chatbot with RAG queries about textbook content
- [x] T043 [SC-004] Verify theme contrast meets WCAG AA standards

## Completed Features Summary

✅ **Documentation UI**: Professional typography, syntax highlighting, responsive layout  
✅ **Sidebar Navigation**: Desktop fixed sidebar, mobile overlay with toggle  
✅ **Theme System**: Light default, dark mode toggle, smooth transitions  
✅ **Dynamic Content**: All modules and sections load from Docusaurus markdown  
✅ **Chatbot**: Floating button, selected text context, mobile-responsive  
✅ **Mobile Optimization**: No horizontal scroll, fixed buttons, smooth navigation

**All user stories (US1, US2, US3) are complete and independently testable.**

