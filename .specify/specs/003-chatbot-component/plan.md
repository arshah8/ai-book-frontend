# Implementation Plan: Chatbot Component

**Branch**: `003-chatbot-component` | **Date**: 2025-01-XX | **Spec**: `.specify/specs/003-chatbot-component/spec.md`  
**Input**: Feature specification for chatbot component with text selection and RAG integration.

## Summary

Implement an interactive RAG-powered chatbot component that allows users to ask questions about textbook content. The chatbot includes text selection context, requires authentication, and is fully mobile-responsive.

## Technical Context

- **Language/Version**: TypeScript 5.x, Next.js 15.5.7  
- **Primary Dependencies**: 
  - `next`, `react`, `react-dom` (Next.js App Router)
  - `@/lib/auth-client` (authentication check)
  - Fetch API for backend communication
- **Storage**: Component state (messages, selected text)  
- **Testing**: Manual testing via browser, text selection, chat interactions  
- **Target Platform**: Web (desktop and mobile browsers)  
- **Constraints**: 
  - Must be mobile-responsive
  - Requires authentication to use
  - Text selection must persist while typing

## Constitution Check

✅ **Documentation-First UI**: Chatbot has clean, professional interface  
✅ **Mobile-First**: Floating button and chat window are mobile-responsive  
✅ **RAG-Integrated Learning**: Chatbot uses RAG for context-aware answers  
✅ **Light Theme Default**: Chatbot supports both light and dark themes

## Project Structure

```text
components/
└── Chatbot.tsx            # Main chatbot component with floating button and chat window

lib/
└── auth-client.ts         # Authentication utilities (getAuthToken, isAuthenticated)
```

**Structure Decision**: Single component handles all chatbot functionality. Authentication check integrated directly. Text selection captured via document events.

## Implementation Phases

### Phase 0: Design & Setup ✅ COMPLETE
- Designed floating chat button interface
- Planned text selection capture mechanism
- Designed authentication requirement
- Planned mobile-responsive layout

### Phase 1: Chatbot Component Structure ✅ COMPLETE
- Created `components/Chatbot.tsx` component
- Implemented floating button (fixed bottom-right)
- Implemented chat window modal
- Added open/close state management

### Phase 2: Authentication Integration ✅ COMPLETE
- Added authentication check using `isAuthenticated()`
- Implemented sign-in prompt for unauthenticated users
- Added links to signin/signup pages
- Showed chat interface only for authenticated users

### Phase 3: Text Selection Capture ✅ COMPLETE
- Added `selectionchange` event listener
- Captured selected text from page
- Displayed selected text in yellow banner
- Preserved selected text while typing (not cleared on input focus)

### Phase 4: Chat API Integration ✅ COMPLETE
- Implemented message sending to `/api/chat` endpoint
- Added Authorization header with JWT token
- Included selected text as context parameter
- Displayed loading states during API calls
- Handled API errors gracefully

### Phase 5: Mobile Responsiveness ✅ COMPLETE
- Made chat button fixed and accessible on mobile
- Adjusted chat window size for mobile (`calc(100vw - 2rem)`)
- Ensured no horizontal scrolling
- Tested on various mobile viewport sizes

## Complexity Tracking

No constitution violations. Chatbot component follows React best practices with clear state management and proper event handling.

