# Feature Specification: Physical AI Book & Documentation UI

**Feature Branch**: `001-physical-ai-docs-ui`  
**Created**: 2025-01-XX  
**Status**: ✅ Complete  
**Input**: User description: "Integrate the Physical AI & Humanoid Robotics textbook, build a documentation-style UI with sidebar, theming, and chatbot, and connect it to the backend RAG."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Read the textbook like documentation (Priority: P1)

A student visits the site and can read the entire Physical AI & Humanoid Robotics textbook in a clean documentation layout with a left-hand sidebar, main content, and persistent navigation.

**Why this priority**: This is the core value of the project – delivering the course content in a usable format.

**Independent Test**:
- Navigate to `/intro`, `/module1`, `/module2`, `/module3`, `/module4`, and `/capstone`.
- Confirm that markdown from the Docusaurus book renders correctly with headings, code blocks, and links.
- Confirm no horizontal scrolling is required on normal desktop and mobile widths.

**Acceptance Scenarios**:

1. **Given** a user opens `/intro`, **When** the page loads, **Then** the introduction markdown from `book/docs/intro` is rendered with proper typography.  
2. **Given** a user navigates between module sections via the sidebar, **When** they click a section, **Then** the corresponding markdown file is rendered and the sidebar highlights the active item.

---

### User Story 2 - Navigate via responsive sidebar (Priority: P1)

On both desktop and mobile, learners can use a documentation-style sidebar to move between modules and sections without losing context.

**Why this priority**: Efficient navigation is critical for a long technical textbook.

**Independent Test**:
- On desktop, verify the sidebar is always visible, fixed to the left, and the main content is aligned with it.
- On mobile, verify a floating hamburger button opens/closes the sidebar and both elements stay visible while scrolling.

**Acceptance Scenarios**:

1. **Given** a small-screen device, **When** the user scrolls a long page, **Then** the sidebar toggle button remains fixed and usable.  
2. **Given** a user opens the sidebar and taps a section, **When** navigation completes, **Then** the sidebar closes and no horizontal scroll bar appears.

---

### User Story 3 - Ask context-aware questions via chatbot (Priority: P2)

Learners can ask questions about the textbook content using a floating chat button that opens an assistant window.

**Why this priority**: The assistant is a major differentiator, turning static content into an interactive learning experience.

**Independent Test**:

- Click the chat button, sign in, and ask questions about a specific module (e.g., URDF or Gazebo).
- Verify the assistant responds with information that matches the textbook sections seeded into Qdrant.
- Select text in the document and open the chat to confirm the selected text appears as context and stays visible while typing.

**Acceptance Scenarios**:

1. **Given** a signed-in user, **When** they open the chatbot and ask a question referencing a topic from the book, **Then** the response cites concepts that are present in the seeded markdown.  
2. **Given** a user selects a paragraph in the docs, **When** they open the chat and start typing, **Then** the “Selected text” strip remains visible until the message is sent or the chat is closed.

---

### Edge Cases

- What happens when markdown contains very long code blocks? → Main content should wrap or scroll within the code block, without causing page-level horizontal scroll.  
- How does the layout handle missing or invalid markdown files? → Should show a friendly 404/“content not found” message instead of crashing.  
- What if the backend RAG or Gemini API is down? → Chat should show a graceful error message without breaking the UI.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The app MUST load markdown content for intro, modules, and capstone from `book/docs` using a reusable markdown loader.  
- **FR-002**: The app MUST render markdown with documentation-style typography and syntax-highlighted code blocks.  
- **FR-003**: The sidebar MUST reflect the course structure (intro, modules 1–4, capstone) and highlight the current route.  
- **FR-004**: On mobile, a fixed toggle button MUST control the sidebar overlay and remain visible while scrolling.  
- **FR-005**: The chatbot button MUST be fixed to the bottom-right on all screens and open a modal chat window.  
- **FR-006**: Selected text from the page MUST be capturable and included as optional context in chat requests.  
- **FR-007**: Theme switching MUST support light and dark modes with a professional, high-contrast palette and light as the default.

### Key Entities *(include if feature involves data)*

- **Markdown Section**: Represents a single markdown file from the book (`module`, `section`, `content`).  
- **Sidebar Item**: Label and route mapping to a specific section; used for navigation and active state.  
- **Chat Message**: User/assistant message with optional `selectedText` context passed to the backend.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All primary textbook routes (`/intro`, `/module1..4`, `/capstone` and their sections) render without runtime errors or 500s.  
- **SC-002**: On a standard mobile viewport (e.g., 390×844), there is no horizontal scroll for any documentation page.  
- **SC-003**: At least 90% of common learner questions about ROS 2, Gazebo, Isaac, and VLA can be answered by the chatbot using RAG context (verified by manual testing).  
- **SC-004**: Theme toggle and navigation controls maintain visual contrast that passes WCAG AA for text and key UI elements in both light and dark modes.


