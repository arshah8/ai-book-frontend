# Deployment Guide

## Option 1: Vercel (Recommended)

Vercel provides the best experience for Next.js apps with API routes.

1. Push your code to GitHub
2. Go to [vercel.com](https://vercel.com) and sign in
3. Click "New Project" and import your repository
4. Add environment variable: `OPENAI_API_KEY`
5. Click "Deploy"

Your app will be live at `https://your-project.vercel.app`

## Option 2: GitHub Pages (Requires Separate Backend)

GitHub Pages only supports static sites. For the chatbot to work, you'll need:

1. Deploy the frontend as static export
2. Deploy the API backend separately (FastAPI on Railway/Render/etc.)
3. Update the chatbot API endpoint in the frontend

### Steps for Static Export:

1. Update `next.config.js`:
```js
const nextConfig = {
  output: 'export',
  images: { unoptimized: true },
}
```

2. Build: `npm run build`
3. Deploy the `out` folder to GitHub Pages

## Option 3: Full Stack with FastAPI Backend

For production with Qdrant and Neon Postgres:

1. Deploy FastAPI backend (see `backend/` directory)
2. Deploy Next.js frontend pointing to backend URL
3. Configure Qdrant and Neon Postgres in backend

## Environment Variables

Required:
- `OPENAI_API_KEY`: Your OpenAI API key

Optional (for production RAG):
- `QDRANT_URL`: Qdrant Cloud URL
- `QDRANT_API_KEY`: Qdrant API key
- `DATABASE_URL`: Neon Postgres connection string

