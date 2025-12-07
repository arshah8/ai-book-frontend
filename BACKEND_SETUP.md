# FastAPI Backend Setup Guide

## Architecture

The project now uses:
- **Frontend**: Next.js 15 (TypeScript)
- **Backend**: FastAPI (Python)
- **Vector DB**: Qdrant Cloud
- **Database**: Neon Postgres
- **AI**: OpenAI SDK (GPT-4o-mini, text-embedding-3-small)

## Backend Setup

### 1. Install Python Dependencies

```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 2. Environment Variables

Create `backend/.env`:

```env
OPENAI_API_KEY=AIzaSyA7Ngz5aBnzv9yIWWZOXp7cfuMtMX9mcJ8
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key
DATABASE_URL=postgresql://user:password@host.neon.tech/dbname?sslmode=require
BETTER_AUTH_SECRET=your_random_secret_key_min_32_chars
```

### 3. Initialize Database

The database tables will be created automatically on first run. Make sure your Neon Postgres database is accessible.

### 4. Seed Qdrant Vectors

After setting up Qdrant, run:

```bash
python scripts/seed_vectors.py
```

This will populate Qdrant with book content embeddings.

### 5. Start Backend Server

```bash
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

The API will be available at `http://localhost:8000`

## Frontend Configuration

Update `.env.local` in the root directory:

```env
BACKEND_URL=http://localhost:8000
# For production, use your deployed backend URL
# BACKEND_URL=https://your-backend.railway.app
```

## API Endpoints

### Chat
- `POST /api/chat` - RAG chatbot
  - Body: `{ "message": "...", "context": "..." }`
  - Headers: `Authorization: Bearer <token>` (optional)

### Translation
- `POST /api/translate` - Translate to Urdu
  - Body: `{ "text": "...", "language": "ur", "module": "..." }`

### Personalization
- `GET /api/personalize` - Get user personalization
  - Headers: `Authorization: Bearer <token>`

### Auth
- `POST /auth/signup` - User signup
- `POST /auth/signin` - User signin

## Deployment

### Backend Deployment (Railway/Render)

1. Push code to GitHub
2. Connect repository to Railway/Render
3. Set environment variables
4. Deploy

### Frontend Deployment (Vercel)

1. Set `BACKEND_URL` environment variable to your backend URL
2. Deploy as usual

## Development Workflow

1. Start backend: `cd backend && uvicorn app.main:app --reload`
2. Start frontend: `npm run dev`
3. Backend runs on port 8000
4. Frontend runs on port 3000
5. Frontend proxies API calls to backend

