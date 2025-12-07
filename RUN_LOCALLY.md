# Running Frontend and Backend Locally

## Quick Start

### Option 1: Using the Run Script (Easiest)

```bash
# Make the script executable (first time only)
chmod +x run-dev.sh

# Run both frontend and backend
./run-dev.sh
```

This will start:
- Backend on http://localhost:8000
- Frontend on http://localhost:3000

### Option 2: Manual Setup

## Step 1: Backend Setup

1. **Navigate to backend directory:**
   ```bash
   cd backend
   ```

2. **Create virtual environment (first time only):**
   ```bash
   python3 -m venv venv
   # If python3 doesn't work, try: python -m venv venv
   ```

3. **Activate virtual environment:**
   ```bash
   # On macOS/Linux:
   source venv/bin/activate
   
   # On Windows:
   venv\Scripts\activate
   ```

4. **Install dependencies (first time only):**
   ```bash
   pip install -r requirements.txt
   ```

5. **Set up environment variables:**
   ```bash
   # Copy example file
   cp env.example .env
   
   # Edit .env and add your API keys:
   # - OPENAI_API_KEY
   # - QDRANT_URL and QDRANT_API_KEY
   # - DATABASE_URL
   # - BETTER_AUTH_SECRET
   ```

6. **Start the backend server:**
   ```bash
   uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
   ```

   Backend will be running at: http://localhost:8000

## Step 2: Frontend Setup

1. **Open a new terminal** (keep backend running)

2. **Navigate to project root:**
   ```bash
   cd /Users/arshah/ai-book
   ```

3. **Install dependencies (first time only):**
   ```bash
   npm install
   ```

4. **Set up environment variables:**
   ```bash
   # Copy example file
   cp env.local.example .env.local
   
   # Edit .env.local and set:
   # BACKEND_URL=http://localhost:8000
   ```

5. **Start the frontend server:**
   ```bash
   npm run dev
   ```

   Frontend will be running at: http://localhost:3000

## Step 3: Verify Everything Works

1. **Check backend health:**
   - Open http://localhost:8000/health
   - Should see: `{"status":"healthy"}`

2. **Check frontend:**
   - Open http://localhost:3000
   - Should see the homepage

3. **Test the chatbot:**
   - Navigate to any module page
   - Click the chatbot icon (bottom right)
   - Ask a question

## Troubleshooting

### Backend Issues

**Port 8000 already in use:**
```bash
# Find and kill the process
lsof -ti:8000 | xargs kill -9

# Or use a different port
uvicorn app.main:app --reload --port 8001
# Then update BACKEND_URL in .env.local
```

**Python dependencies not found:**
```bash
# Make sure virtual environment is activated
source venv/bin/activate  # or venv\Scripts\activate on Windows
pip install -r requirements.txt
# If pip doesn't work, try: pip3 install -r requirements.txt
```

**Python command not found:**
```bash
# On macOS, use python3 instead of python
python3 -m venv venv
source venv/bin/activate
pip3 install -r requirements.txt
# Or install Python from python.org or using Homebrew: brew install python3
```

**Database connection error:**
- Check your `DATABASE_URL` in `backend/.env`
- Make sure Neon database is running
- Verify connection string format

### Frontend Issues

**Port 3000 already in use:**
```bash
# Find and kill the process
lsof -ti:3000 | xargs kill -9

# Or use a different port
npm run dev -- -p 3001
```

**Backend connection error:**
- Check `BACKEND_URL` in `.env.local` is `http://localhost:8000`
- Make sure backend is running
- Check CORS settings in backend

**Module not found errors:**
```bash
# Reinstall dependencies
rm -rf node_modules package-lock.json
npm install
```

## Development Workflow

1. **Terminal 1 - Backend:**
   ```bash
   cd backend
   python3 -m venv venv  # First time only
   source venv/bin/activate
   pip3 install -r requirements.txt  # First time only
   uvicorn app.main:app --reload
   ```

2. **Terminal 2 - Frontend:**
   ```bash
   npm run dev
   ```

3. **Make changes:**
   - Backend auto-reloads on file changes (--reload flag)
   - Frontend auto-reloads via Next.js hot reload

## API Documentation

Once backend is running, visit:
- Swagger UI: http://localhost:8000/docs
- ReDoc: http://localhost:8000/redoc

## Stopping Services

- **Backend:** Press `Ctrl+C` in the backend terminal
- **Frontend:** Press `Ctrl+C` in the frontend terminal

