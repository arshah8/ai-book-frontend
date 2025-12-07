# Tasks: Chatbot Component

**Input**: `spec.md` and `plan.md` under `.specify/specs/003-chatbot-component/`  
**Prerequisites**: Backend chat API (`/api/chat`) and authentication system must be available.

## Phase 1: Component Structure (DONE)

- [x] T001 [P] Create `components/Chatbot.tsx` component
- [x] T002 [P] Add state for isOpen, messages, input, selectedText, isLoading
- [x] T003 [P] Implement floating chat button (fixed bottom-right)
- [x] T004 [P] Implement chat window modal that opens/closes
- [x] T005 [P] Add toggle functionality for opening/closing chat

## Phase 2: Authentication Integration (DONE)

- [x] T010 [US1] Import `getAuthToken`, `isAuthenticated` from auth-client
- [x] T011 [US1] Add isAuthenticated state
- [x] T012 [US1] Check authentication on component mount (useEffect)
- [x] T013 [US1] Display sign-in prompt when not authenticated
- [x] T014 [US1] Add links to signin/signup pages in prompt
- [x] T015 [US1] Show chat interface only when authenticated
- [x] T016 [US1] Listen for storage changes to update auth state

## Phase 3: Text Selection Capture (DONE)

- [x] T020 [US2] Add `selectionchange` event listener in useEffect
- [x] T021 [US2] Capture selected text from window.getSelection()
- [x] T022 [US2] Update selectedText state when text is selected
- [x] T023 [US2] Display selected text in yellow banner above chat input
- [x] T024 [US2] Preserve selected text while typing (don't clear on input focus)
- [x] T025 [US2] Clear selected text when message is sent
- [x] T026 [US2] Clear selected text when chat window is closed

## Phase 4: Chat API Integration (DONE)

- [x] T030 [US1] Implement handleSend function
- [x] T031 [US1] Check authentication before sending
- [x] T032 [US1] Build request body with message and context (selected text)
- [x] T033 [US1] Add Authorization header with JWT token
- [x] T034 [US1] Call `/api/chat` endpoint (via Next.js API route or direct backend)
- [x] T035 [US1] Display loading indicator during API call
- [x] T036 [US1] Add assistant response to messages state
- [x] T037 [US1] Handle API errors with error message display
- [x] T038 [US1] Auto-scroll to latest message when new message arrives

## Phase 5: Message Display (DONE)

- [x] T040 [US1] Display messages in chat window
- [x] T041 [US1] Style user messages (right-aligned, indigo background)
- [x] T042 [US1] Style assistant messages (left-aligned, gray background)
- [x] T043 [US1] Show empty state message when no messages
- [x] T044 [US1] Add loading animation (three dots) during API call

## Phase 6: Mobile Responsiveness (DONE)

- [x] T050 [US3] Make chat button responsive (smaller on mobile, larger on desktop)
- [x] T051 [US3] Adjust chat window width for mobile (`calc(100vw - 2rem)`)
- [x] T052 [US3] Adjust chat window height for mobile (500px on mobile, 600px on desktop)
- [x] T053 [US3] Ensure chat button remains fixed and accessible on scroll
- [x] T054 [US3] Test on mobile viewports (320px+)
- [x] T055 [US3] Verify no horizontal scrolling occurs

## Phase 7: Styling & Polish (DONE)

- [x] T060 [P] Add dark mode support to chatbot
- [x] T061 [P] Style chat window with proper borders and shadows
- [x] T062 [P] Add smooth transitions for opening/closing
- [x] T063 [P] Style input field and send button
- [x] T064 [P] Ensure selected text banner is clearly visible

## Completed Features Summary

✅ **RAG Integration**: Chatbot uses backend RAG for context-aware answers  
✅ **Text Selection**: Selected text captured and included as context  
✅ **Authentication**: Chatbot requires sign-in, shows prompt for unauthenticated users  
✅ **Mobile Responsive**: Fully functional on mobile devices  
✅ **Error Handling**: Graceful error messages for API failures  
✅ **Loading States**: Clear loading indicators during API calls

**All user stories (US1, US2, US3) are complete and independently testable.**

