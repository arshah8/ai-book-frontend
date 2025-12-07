# Physical AI Frontend Constitution

## Core Principles

### I. Documentation-First UI
Every page should feel like professional technical documentation. Typography, spacing, and navigation patterns should prioritize readability and clarity over flashy animations. The UI should get out of the way and let the content shine.

### II. Mobile-First Responsive Design
All components must work seamlessly on mobile devices. Sidebars collapse, buttons remain accessible while scrolling, and no horizontal scrolling should occur. Mobile users are first-class citizens.

### III. Light Theme Default
The application defaults to light theme for better accessibility and professional appearance. Dark theme is available but not forced. Theme switching must be instant and preserve user preference.

### IV. RAG-Integrated Learning
The chatbot is not an afterthought—it's a core learning tool. Selected text from the documentation should seamlessly flow into chat context. The assistant must answer questions using the actual textbook content via RAG.

### V. Docusaurus Content Integration
The book content lives in `book/docs/` as markdown. The frontend must dynamically load and render this content without hardcoding. When the book updates, the frontend should reflect those changes automatically.

### VI. Progressive Enhancement
Core functionality (reading the book, navigation) works without JavaScript. Enhanced features (chatbot, theme switching) gracefully degrade if unavailable.

## Technology Stack

**Framework**: Next.js 15+ (App Router)  
**Styling**: Tailwind CSS with custom documentation typography  
**Markdown**: react-markdown with remarkGfm, rehypeHighlight  
**Theming**: next-themes with class-based dark mode  
**Content Source**: Docusaurus markdown files in `book/docs/`

## Development Workflow

- All new features must be documented in `specs/` using Specify templates
- Mobile responsiveness must be tested on actual devices or browser dev tools
- Theme changes must be verified in both light and dark modes
- Chatbot integration must be tested with actual backend RAG responses

## Governance

This constitution guides all frontend development decisions. When in doubt, prioritize:
1. User experience (readability, navigation, accessibility)
2. Content accuracy (RAG integration, markdown rendering)
3. Performance (fast page loads, smooth transitions)
4. Maintainability (clear code structure, reusable components)

**Version**: 1.0.0 | **Ratified**: 2025-01-XX | **Last Amended**: 2025-01-XX
