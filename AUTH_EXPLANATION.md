# Authentication Explanation

## ⚠️ Important: BETTER_AUTH_SECRET is Server-Side Only!

**BETTER_AUTH_SECRET** is a **server-side secret** used to sign JWT tokens. It should **NEVER** be sent from the frontend!

## How Authentication Works

### 1. Server-Side (Backend)
- `BETTER_AUTH_SECRET` stays in `backend/.env`
- Used to **sign** JWT tokens when users sign in
- Used to **verify** JWT tokens when users make requests

### 2. Client-Side (Frontend)
- User signs up/signs in → Gets a **JWT token**
- Frontend stores the **JWT token** in localStorage
- Frontend sends the **JWT token** (not the secret!) in requests

## Authentication Flow

```
1. User visits /signup or /signin
2. User enters email/password
3. Frontend sends request to backend
4. Backend creates JWT token using BETTER_AUTH_SECRET
5. Backend returns JWT token to frontend
6. Frontend stores JWT token in localStorage
7. Frontend sends JWT token in Authorization header for future requests
8. Backend verifies JWT token using BETTER_AUTH_SECRET
```

## What Frontend Sends

✅ **JWT Token** (after signin):
```
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

❌ **NOT** the BETTER_AUTH_SECRET!

## Files Created

1. ✅ `app/signup/page.tsx` - Signup page with background questionnaire
2. ✅ `app/signin/page.tsx` - Signin page
3. ✅ `lib/auth-client.ts` - Client-side auth utilities
4. ✅ Backend auth endpoints (`/auth/signup`, `/auth/signin`)

## Usage

### Sign Up
1. Go to http://localhost:3000/signup
2. Fill in email, password, and background info
3. Submit → Gets JWT token → Redirects to home

### Sign In
1. Go to http://localhost:3000/signin
2. Enter email and password
3. Submit → Gets JWT token → Redirects to home

### Using Auth in Components

```typescript
import { getAuthToken, getUser, isAuthenticated } from '@/lib/auth-client';

// Check if user is logged in
if (isAuthenticated()) {
  const user = getUser();
  const token = getAuthToken();
  // Use token in API calls
}
```

## Security Notes

- ✅ BETTER_AUTH_SECRET stays on server
- ✅ JWT tokens expire after 7 days
- ✅ Tokens are verified on every request
- ⚠️ In production, hash passwords (currently stored plain text for demo)

