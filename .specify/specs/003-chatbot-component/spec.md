# Feature Specification: Chatbot Component

**Feature Branch**: `003-chatbot-component`  
**Created**: 2025-01-XX  
**Status**: ✅ Complete  
**Input**: User description: "Chatbot component with text selection and RAG integration"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Interactive RAG-Powered Chatbot (Priority: P1)

A user can open a floating chatbot interface, ask questions about the textbook content, and receive context-aware answers using RAG (Retrieval-Augmented Generation) from the backend. The chatbot requires authentication and shows a sign-in prompt for unauthenticated users.

**Why this priority**: The chatbot is a core learning tool that makes the static textbook interactive. RAG ensures answers are based on actual textbook content, not generic responses.

**Independent Test**: Can be fully tested by:
- Signing in as a user
- Clicking the floating chat button
- Asking a question about textbook content
- Verifying response is relevant and based on textbook material

**Acceptance Scenarios**:

1. **Given** an authenticated user, **When** they open the chatbot and ask "What is ROS 2?", **Then** the chatbot returns an answer based on textbook content about ROS 2.
2. **Given** an unauthenticated user, **When** they click the chat button, **Then** a sign-in prompt is displayed with links to sign in/sign up.
3. **Given** a user asks a question, **When** the backend processes it, **Then** a loading indicator is shown and response appears when ready.

---

### User Story 2 - Text Selection Context (Priority: P1)

A user can select text from any page in the textbook, and when they open the chatbot, the selected text is automatically included as context for their question. The selected text remains visible while typing.

**Why this priority**: Text selection enables users to ask specific questions about content they're reading, making the learning experience more contextual and efficient.

**Independent Test**: Can be fully tested by:
- Selecting text on a page
- Opening the chatbot
- Verifying selected text is shown in a yellow banner
- Typing a question and verifying selected text persists
- Sending message and verifying context is included

**Acceptance Scenarios**:

1. **Given** a user selects text on a page, **When** they open the chatbot, **Then** the selected text is displayed in a yellow banner above the chat input.
2. **Given** selected text is visible in the chatbot, **When** the user types a question, **Then** the selected text remains visible until the message is sent or chat is closed.
3. **Given** a user sends a message with selected text, **When** the request is sent, **Then** the selected text is included as context in the API request to backend.

---

### User Story 3 - Mobile-Responsive Chat Interface (Priority: P2)

The chatbot interface is fully responsive, with a floating button that remains accessible on mobile devices, and a chat window that adapts to screen size without causing horizontal scrolling.

**Why this priority**: Mobile users are a significant portion of learners. The chatbot must be accessible and usable on all devices.

**Independent Test**: Can be fully tested by:
- Opening the app on mobile viewport
- Verifying chat button is visible and accessible
- Opening chat and verifying window fits screen
- Testing scrolling and input on mobile

**Acceptance Scenarios**:

1. **Given** a mobile user, **When** they scroll the page, **Then** the chat button remains fixed and accessible at bottom-right.
2. **Given** a mobile user opens the chat, **When** the window appears, **Then** it uses full width minus margins (`calc(100vw - 2rem)`) and appropriate height.
3. **Given** a mobile user types a message, **When** the keyboard appears, **Then** the chat window adjusts appropriately without breaking layout.

---

### Edge Cases

- What happens when backend API fails? → Error message is shown in chat, user can retry.
- How is selected text cleared? → Selected text is cleared when message is sent or chat window is closed.
- What if user selects text while chat is already open? → New selection updates the selected text banner.
- How are long messages handled? → Messages wrap properly, chat scrolls to show latest messages.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a floating chat button fixed at bottom-right that opens/closes the chat interface.
- **FR-002**: System MUST require authentication to use the chatbot, showing sign-in prompt for unauthenticated users.
- **FR-003**: System MUST capture text selections from the page using `selectionchange` event.
- **FR-004**: System MUST display selected text in a yellow banner above chat input when present.
- **FR-005**: System MUST preserve selected text while user types (not clear on input focus).
- **FR-006**: System MUST send selected text as `context` parameter in chat API requests.
- **FR-007**: System MUST call `/api/chat` endpoint with message, context, and Authorization header.
- **FR-008**: System MUST display loading indicator during API requests.
- **FR-009**: System MUST be mobile-responsive with fixed button and adaptive chat window sizing.
- **FR-010**: System MUST auto-scroll to latest message when new messages arrive.

### Key Entities *(include if feature involves data)*

- **Message**: Chat message with role ('user' | 'assistant') and content (string).
- **SelectedText**: String containing user-selected text from the page, used as context for questions.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Chatbot opens/closes within 200ms of button click (smooth animation).
- **SC-002**: Text selection is captured and displayed within 100ms of selection.
- **SC-003**: Chat API requests complete within 5 seconds for typical questions.
- **SC-004**: Chatbot is fully functional on mobile viewports (320px+) without horizontal scrolling.
- **SC-005**: Selected text persists correctly while typing in 100% of test cases.
