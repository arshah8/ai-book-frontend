# Implementation Plan: Physical AI Book & Documentation UI

**Branch**: `001-physical-ai-docs-ui` | **Date**: 2025-01-XX | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-physical-ai-docs-ui/spec.md`

## Summary

Build a documentation-style frontend for the Physical AI & Humanoid Robotics textbook that integrates Docusaurus markdown content, provides responsive navigation via a sidebar, supports light/dark theming, and includes a RAG-powered chatbot for interactive learning.

## Technical Context

**Language/Version**: TypeScript 5.x, Next.js 15.5.7  
**Primary Dependencies**: 
- `next`, `react`, `react-dom` (Next.js App Router)
- `react-markdown`, `remark-gfm`, `rehype-highlight` (Markdown rendering)
- `next-themes` (Theme management)
- `tailwindcss` (Styling)

**Storage**: File-based (markdown in `book/docs/`)  
**Testing**: Manual testing, browser dev tools for responsive design  
**Target Platform**: Web (desktop and mobile browsers)  
**Project Type**: Web application (Next.js SPA)  
**Performance Goals**: 
- First Contentful Paint < 1.5s
- No layout shift on navigation
- Smooth theme transitions (< 200ms)

**Constraints**: 
- Must work on mobile viewports (320px+)
- Must support both light and dark themes with WCAG AA contrast
- No horizontal scrolling on any page

**Scale/Scope**: 
- ~30 markdown pages (intro, 4 modules with sections, capstone)
- Single-user learning experience
- Chatbot handles concurrent requests via backend

## Constitution Check

✅ **Documentation-First UI**: All pages use professional typography and spacing  
✅ **Mobile-First**: Sidebar collapses, buttons fixed, no horizontal scroll  
✅ **Light Theme Default**: Configured in ThemeProvider  
✅ **RAG-Integrated**: Chatbot uses selected text and backend RAG  
✅ **Docusaurus Integration**: Dynamic markdown loading from `book/docs/`

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-ai-docs-ui/
├── spec.md              # Feature specification
├── plan.md              # This file
└── tasks.md             # Implementation tasks
```

### Source Code (repository root)

```text
app/
├── layout.tsx           # Root layout with ThemeProvider, Navbar
├── page.tsx             # Home page with module cards
├── intro/
│   └── page.tsx         # Introduction page
├── module1/
│   ├── page.tsx         # Module 1 overview
│   └── [section]/
│       └── page.tsx     # Dynamic section pages
├── module2/             # Same structure as module1
├── module3/
├── module4/
└── capstone/

components/
├── Navbar.tsx           # Top navigation with theme switcher
├── Sidebar.tsx          # Documentation sidebar (desktop + mobile)
├── DocsLayout.tsx       # Layout wrapper with sidebar
├── MobileSidebarToggle.tsx  # Mobile hamburger button
├── MarkdownContent.tsx  # Markdown renderer with syntax highlighting
├── Chatbot.tsx          # Floating chat button + modal
└── ThemeSwitcher.tsx    # Light/dark toggle

lib/
└── markdown-loader.ts   # Utility to load markdown from book/docs/

book/
└── docs/                # Docusaurus markdown content
    ├── intro.md
    ├── module1/
    ├── module2/
    ├── module3/
    ├── module4/
    └── capstone/
```

**Structure Decision**: Next.js App Router with dynamic routes for modules/sections. Components are reusable and mobile-responsive. Markdown is loaded server-side for SEO and performance.

## Implementation Phases

### Phase 0: Research & Design ✅ COMPLETE
- Analyzed Docusaurus markdown structure
- Designed sidebar navigation pattern
- Selected markdown rendering libraries
- Planned theme system architecture

### Phase 1: Core Infrastructure ✅ COMPLETE
- Set up Next.js App Router structure
- Created `markdown-loader.ts` utility
- Implemented `MarkdownContent` component with syntax highlighting
- Configured Tailwind with custom typography

### Phase 2: Navigation & Layout ✅ COMPLETE
- Built `Sidebar` component with collapsible sections
- Created `DocsLayout` wrapper
- Implemented `MobileSidebarToggle` with fixed positioning
- Added active route highlighting

### Phase 3: Theming ✅ COMPLETE
- Integrated `next-themes` with `ThemeProvider`
- Created `ThemeSwitcher` component
- Configured light theme as default
- Styled all components for both themes

### Phase 4: Chatbot Integration ✅ COMPLETE
- Built floating chat button (fixed, mobile-responsive)
- Implemented chat modal with authentication check
- Added selected text capture and context preservation
- Connected to backend `/api/chat` endpoint

### Phase 5: Content Integration ✅ COMPLETE
- Created dynamic routes for all modules/sections
- Implemented markdown loading for intro, modules, capstone
- Added Table of Contents generation
- Ensured all pages render without errors

### Phase 6: Polish & Mobile Optimization ✅ COMPLETE
- Fixed horizontal scrolling issues
- Ensured sidebar closes on navigation
- Made all buttons fixed/sticky on mobile
- Verified theme switching works correctly

## Complexity Tracking

No constitution violations. All features align with core principles.

