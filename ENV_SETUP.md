# Environment Variables Setup Guide

## Frontend (.env.local)

Create `.env.local` in the root directory:

```env
BACKEND_URL=http://localhost:8000
```

**For Production:**
- Update `BACKEND_URL` to your deployed backend URL (e.g., Railway, Render)

## Backend (.env)

Create `.env` in the `backend/` directory with:

### Required Variables

1. **OPENAI_API_KEY**
   - Get from: https://platform.openai.com/api-keys
   - Used for: Chat responses, embeddings, translations

2. **QDRANT_URL**
   - Get from: https://cloud.qdrant.io (Free tier available)
   - Format: `https://your-cluster-id.qdrant.io`
   - Used for: Vector search

3. **QDRANT_API_KEY**
   - Get from: Qdrant Cloud dashboard
   - Used for: Authenticating with Qdrant

4. **DATABASE_URL**
   - Get from: https://neon.tech (Free tier available)
   - Format: `postgresql://user:password@host.neon.tech/dbname?sslmode=require`
   - Used for: User profiles, chat history, translations cache

5. **BETTER_AUTH_SECRET**
   - Generate a random 32+ character string
   - Command: `openssl rand -hex 32`
   - Used for: JWT token signing

### Optional Variables

- `HOST=0.0.0.0` - Server host (default: 0.0.0.0)
- `PORT=8000` - Server port (default: 8000)

## Quick Setup

1. **Frontend:**
   ```bash
   cp .env.example .env.local
   # Edit .env.local and add BACKEND_URL
   ```

2. **Backend:**
   ```bash
   cd backend
   cp .env.example .env
   # Edit .env and add all API keys
   ```

## Getting API Keys

### OpenAI
1. Go to https://platform.openai.com/api-keys
2. Create new secret key
3. Copy and paste into `OPENAI_API_KEY`

### Qdrant Cloud
1. Sign up at https://cloud.qdrant.io
2. Create a free cluster
3. Copy cluster URL → `QDRANT_URL`
4. Copy API key → `QDRANT_API_KEY`

### Neon Postgres
1. Sign up at https://neon.tech
2. Create a new project
3. Copy connection string → `DATABASE_URL`
4. Format: `postgresql://user:password@host.neon.tech/dbname?sslmode=require`

### Better Auth Secret
Generate a secure random string:
```bash
openssl rand -hex 32
```
Or use an online generator and copy to `BETTER_AUTH_SECRET`

## Security Notes

- ⚠️ Never commit `.env` or `.env.local` to git
- ✅ Both files are in `.gitignore`
- ✅ Use `.env.example` files as templates
- ✅ For production, set environment variables in your hosting platform

## Verification

After setting up, verify your backend:

```bash
cd backend
python -c "from dotenv import load_dotenv; import os; load_dotenv(); print('OPENAI_API_KEY:', 'SET' if os.getenv('OPENAI_API_KEY') else 'MISSING')"
```

