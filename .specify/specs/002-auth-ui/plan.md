# Implementation Plan: Authentication UI

**Branch**: `002-auth-ui` | **Date**: 2025-01-XX | **Spec**: `.specify/specs/002-auth-ui/spec.md`  
**Input**: Feature specification for authentication UI with signup and signin pages.

## Summary

Implement user authentication UI with signup and signin pages that integrate with the backend authentication API. The UI collects user information including background questionnaire, manages authentication state, and provides seamless user experience.

## Technical Context

- **Language/Version**: TypeScript 5.x, Next.js 15.5.7  
- **Primary Dependencies**: 
  - `next`, `react`, `react-dom` (Next.js App Router)
  - `@/lib/auth-client` (authentication client utilities)
  - `localStorage` (token and user state persistence)
- **Storage**: localStorage for authentication tokens and user data  
- **Testing**: Manual testing via browser, form submission, navigation  
- **Target Platform**: Web (desktop and mobile browsers)  
- **Constraints**: 
  - Must work on mobile viewports
  - Authentication state must persist across page refreshes
  - Forms must validate before submission

## Constitution Check

✅ **Documentation-First UI**: Auth pages use clean, professional forms  
✅ **Mobile-First**: Forms are responsive and work on mobile devices  
✅ **Light Theme Default**: Auth pages support both light and dark themes  
✅ **RAG-Integrated**: Authentication enables RAG chatbot access

## Project Structure

```text
app/
├── signup/
│   └── page.tsx           # Signup page with background questionnaire
├── signin/
│   └── page.tsx           # Signin page
└── components/
    └── Navbar.tsx         # Shows auth state and sign in/out links

lib/
└── auth-client.ts         # Authentication utilities (getAuthToken, isAuthenticated, etc.)
```

**Structure Decision**: Separate pages for signup and signin. Auth state managed via localStorage and auth-client utilities. Navbar shows authentication status.

## Implementation Phases

### Phase 0: Design & Setup ✅ COMPLETE
- Designed signup form with background questionnaire
- Designed signin form layout
- Planned authentication state management
- Designed integration with backend API

### Phase 1: Auth Client Utilities ✅ COMPLETE
- Created `lib/auth-client.ts` with authentication functions
- Implemented `getAuthToken()` to retrieve token from localStorage
- Implemented `isAuthenticated()` to check auth status
- Implemented `getUser()` to get user data
- Implemented `signOut()` to clear auth state

### Phase 2: Signup Page ✅ COMPLETE
- Created `app/signup/page.tsx` with registration form
- Added form fields: email, password, name, software_background, hardware_background, experience_level
- Implemented form submission to `/auth/signup` endpoint
- Added error handling and loading states
- Implemented redirect to home page on success
- Added localStorage token storage

### Phase 3: Signin Page ✅ COMPLETE
- Created `app/signin/page.tsx` with signin form
- Added form fields: email, password
- Implemented form submission to `/auth/signin` endpoint
- Added error handling and loading states
- Implemented redirect to home page on success
- Added localStorage token storage

### Phase 4: Navbar Integration ✅ COMPLETE
- Updated `components/Navbar.tsx` to show auth state
- Added user name/email display when authenticated
- Added sign out button for authenticated users
- Added sign in/sign up links for unauthenticated users
- Implemented auth state persistence on page refresh

## Complexity Tracking

No constitution violations. Authentication UI follows standard Next.js patterns with clear separation between pages and utilities.

