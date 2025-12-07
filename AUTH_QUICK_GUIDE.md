# Authentication Quick Guide

## ✅ Answer: NO, Frontend Does NOT Send BETTER_AUTH_SECRET

**BETTER_AUTH_SECRET** is **server-side only** - it stays in `backend/.env` and is never sent to the frontend!

## What Frontend Sends Instead

After signing in, the frontend sends a **JWT Token** (not the secret):

```
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

## How It Works

1. **User signs up/signs in** → Frontend calls `/auth/signup` or `/auth/signin`
2. **Backend creates JWT** → Uses `BETTER_AUTH_SECRET` to sign the token
3. **Backend returns JWT** → Sends token to frontend
4. **Frontend stores JWT** → Saves in localStorage
5. **Frontend uses JWT** → Sends in `Authorization: Bearer <token>` header

## Pages Created

- ✅ `/signup` - Sign up with background questionnaire
- ✅ `/signin` - Sign in page
- ✅ Homepage shows sign in/sign up links

## Test It

1. Go to http://localhost:3000/signup
2. Create an account
3. You'll be redirected to home
4. The chatbot will automatically use your auth token

## Security

- ✅ Secret stays on server
- ✅ Only JWT tokens sent from frontend
- ✅ Tokens expire after 7 days
- ✅ Tokens verified on every request

No need to manually send anything - it's all automatic!

