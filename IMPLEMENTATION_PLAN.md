# Bonus Features Implementation Plan

## Overview
This document outlines the step-by-step plan to implement all bonus features for the hackathon.

## Features to Implement

### 1. Better-auth for Signup/Signin (50 points)
**Files to Create/Modify:**
- `lib/auth.ts` - Better-auth configuration
- `app/api/auth/[...all]/route.ts` - Auth API routes
- `app/signup/page.tsx` - Signup page with background questionnaire
- `app/signin/page.tsx` - Signin page
- `components/AuthButton.tsx` - Auth UI component
- `components/UserProfile.tsx` - User profile display
- `app/layout.tsx` - Add auth provider
- `package.json` - Add better-auth dependency

**Database Schema:**
- Users table (via Better-auth)
- User profiles table with:
  - software_background (text)
  - hardware_background (text)
  - experience_level (enum)
  - preferred_language (enum)

### 2. Content Personalization (50 points)
**Files to Create/Modify:**
- `lib/personalization.ts` - Personalization logic
- `components/PersonalizeButton.tsx` - Personalization toggle
- `app/[module]/page.tsx` - Update all module pages
- `app/intro/page.tsx` - Add personalization
- `app/capstone/page.tsx` - Add personalization
- `app/api/personalize/route.ts` - Personalization API

**Logic:**
- Show/hide advanced topics based on experience
- Adjust code examples complexity
- Provide beginner/intermediate/advanced content variants
- Store preferences in user profile

### 3. Urdu Translation (50 points)
**Files to Create/Modify:**
- `lib/translation.ts` - Translation utilities
- `components/TranslateButton.tsx` - Translation toggle
- `app/[module]/page.tsx` - Add translation support
- `app/intro/page.tsx` - Add translation
- `app/capstone/page.tsx` - Add translation
- `app/api/translate/route.ts` - Translation API (OpenAI)
- `lib/urdu-content.ts` - Pre-translated content (optional cache)

**Implementation:**
- Use OpenAI for translation
- Cache translations in database
- Toggle between English/Urdu
- Preserve formatting and code blocks

### 4. Qdrant + Neon Postgres Upgrade (50 points)
**Files to Create/Modify:**
- `lib/qdrant.ts` - Qdrant client setup
- `lib/database.ts` - Neon Postgres connection
- `lib/embeddings.ts` - Enhanced embedding management
- `app/api/chat/route.ts` - Update to use Qdrant
- `scripts/seed-vectors.ts` - Script to seed Qdrant with book content
- `package.json` - Add @qdrant/qdrant-js and @neondatabase/serverless
- `.env.local.example` - Update with new env vars

**Database Schema:**
- `chunks` table: id, content, module, section, embedding_id
- `user_chat_history` table: id, user_id, message, response, timestamp
- Vector collection in Qdrant: `book_content`

## Implementation Order

1. **Phase 1: Database & Auth Setup**
   - Set up Neon Postgres
   - Configure Better-auth
   - Create database schema
   - Implement signup/signin

2. **Phase 2: Qdrant Integration**
   - Set up Qdrant Cloud
   - Create vector collection
   - Seed book content
   - Update RAG chatbot

3. **Phase 3: User Features**
   - Add personalization logic
   - Implement translation
   - Update all pages
   - Add UI components

4. **Phase 4: Testing & Polish**
   - Test all features
   - Update documentation
   - Fix any issues

## Environment Variables Needed

```env
# Existing
OPENAI_API_KEY=...

# New
DATABASE_URL=postgresql://... (Neon)
QDRANT_URL=https://...
QDRANT_API_KEY=...
BETTER_AUTH_SECRET=...
BETTER_AUTH_URL=http://localhost:3000
```

## Dependencies to Add

```json
{
  "better-auth": "^1.0.0",
  "@qdrant/qdrant-js": "^1.0.0",
  "@neondatabase/serverless": "^0.5.0",
  "drizzle-orm": "^0.29.0",
  "postgres": "^3.4.0"
}
```

## File Structure After Implementation

```
ai-book/
├── app/
│   ├── api/
│   │   ├── auth/[...all]/route.ts
│   │   ├── chat/route.ts (updated)
│   │   ├── personalize/route.ts
│   │   └── translate/route.ts
│   ├── signup/page.tsx
│   ├── signin/page.tsx
│   └── [all existing pages updated]
├── components/
│   ├── AuthButton.tsx
│   ├── PersonalizeButton.tsx
│   ├── TranslateButton.tsx
│   ├── UserProfile.tsx
│   └── Chatbot.tsx (updated)
├── lib/
│   ├── auth.ts
│   ├── database.ts
│   ├── qdrant.ts
│   ├── embeddings.ts
│   ├── personalization.ts
│   ├── translation.ts
│   └── urdu-content.ts
├── scripts/
│   └── seed-vectors.ts
└── [config files]
```

## Success Criteria

- ✅ Users can sign up with background questionnaire
- ✅ Users can sign in/out
- ✅ Content personalizes based on user background
- ✅ Content can be translated to Urdu
- ✅ RAG chatbot uses Qdrant for vector search
- ✅ Chat history stored in Neon Postgres
- ✅ All features work together seamlessly

