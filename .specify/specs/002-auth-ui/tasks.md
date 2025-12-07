# Tasks: Authentication UI

**Input**: `spec.md` and `plan.md` under `.specify/specs/002-auth-ui/`  
**Prerequisites**: Backend authentication API (002-user-authentication) must be available.

## Phase 1: Auth Client Utilities (DONE)

- [x] T001 [P] Create `lib/auth-client.ts` file
- [x] T002 [P] Implement `getAuthToken()` function to read from localStorage
- [x] T003 [P] Implement `isAuthenticated()` function to check token existence
- [x] T004 [P] Implement `getUser()` function to get user data from localStorage
- [x] T005 [P] Implement `signOut()` function to clear token and user data

## Phase 2: Signup Page (DONE)

- [x] T010 [US1] Create `app/signup/page.tsx` component
- [x] T011 [US1] Add form with email field (required, type="email")
- [x] T012 [US1] Add form with password field (required, type="password")
- [x] T013 [US1] Add form with name field (optional, type="text")
- [x] T014 [US1] Add form with software_background textarea (optional)
- [x] T015 [US1] Add form with hardware_background textarea (optional)
- [x] T016 [US1] Add form with experience_level select (beginner, intermediate, advanced)
- [x] T017 [US1] Implement form state management with useState
- [x] T018 [US1] Implement form submission handler
- [x] T019 [US1] Call backend `/auth/signup` endpoint on form submit
- [x] T020 [US1] Store auth_token and user in localStorage on success
- [x] T021 [US1] Redirect to home page (`/`) on successful signup
- [x] T022 [US1] Display error messages for failed signup
- [x] T023 [US1] Add loading state during form submission
- [x] T024 [US1] Add link to signin page for existing users

## Phase 3: Signin Page (DONE)

- [x] T030 [US2] Create `app/signin/page.tsx` component
- [x] T031 [US2] Add form with email field (required, type="email")
- [x] T032 [US2] Add form with password field (required, type="password")
- [x] T033 [US2] Implement form state management with useState
- [x] T034 [US2] Implement form submission handler
- [x] T035 [US2] Call backend `/auth/signin` endpoint on form submit
- [x] T036 [US2] Store auth_token and user in localStorage on success
- [x] T037 [US2] Redirect to home page (`/`) on successful signin
- [x] T038 [US2] Display error messages for failed signin
- [x] T039 [US2] Add loading state during form submission
- [x] T040 [US2] Add link to signup page for new users

## Phase 4: Navbar Integration (DONE)

- [x] T050 [US3] Update `components/Navbar.tsx` to check authentication state
- [x] T051 [US3] Import `isAuthenticated`, `getUser`, `signOut` from auth-client
- [x] T052 [US3] Add useState for user data and mounted state
- [x] T053 [US3] Check auth state on component mount (useEffect)
- [x] T054 [US3] Display user name/email when authenticated
- [x] T055 [US3] Add sign out button that calls signOut() function
- [x] T056 [US3] Display sign in/sign up links when not authenticated
- [x] T057 [US3] Handle auth state persistence on page refresh

## Phase 5: Styling & Polish (DONE)

- [x] T060 [P] Style signup and signin pages with consistent design
- [x] T061 [P] Add dark mode support to auth pages
- [x] T062 [P] Make forms mobile-responsive
- [x] T063 [P] Add proper form validation (required fields, email format)
- [x] T064 [P] Ensure error messages are clearly visible

## Completed Features Summary

✅ **Signup Flow**: Complete registration with background questionnaire  
✅ **Signin Flow**: Simple email/password authentication  
✅ **State Management**: Auth state persists across page refreshes  
✅ **Navbar Integration**: User info and sign out displayed when authenticated  
✅ **Error Handling**: Clear error messages for failed authentication  
✅ **Mobile Responsive**: Forms work on all device sizes

**All user stories (US1, US2, US3) are complete and independently testable.**

