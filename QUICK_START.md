# Quick Start Guide

## 🚀 Run Everything with One Command

```bash
./run-dev.sh
```

This will:
- ✅ Check and create .env files if needed
- ✅ Set up Python virtual environment
- ✅ Install all dependencies
- ✅ Start backend on http://localhost:8000
- ✅ Start frontend on http://localhost:3000

## 📋 Prerequisites

1. **Node.js 18+** installed
2. **Python 3.10+** installed
3. **API Keys** ready (see ENV_SETUP.md)

## 🔑 First Time Setup

1. **Set up environment files:**
   ```bash
   # Frontend
   cp env.local.example .env.local
   # Edit .env.local and set BACKEND_URL=http://localhost:8000
   
   # Backend
   cd backend
   cp env.example .env
   # Edit .env and add your API keys
   ```

2. **Run the setup script:**
   ```bash
   ./run-dev.sh
   ```

## 🎯 Manual Setup (If Needed)

### Backend Only
```bash
cd backend
python3 -m venv venv  # Use python3 on macOS
source venv/bin/activate  # Windows: venv\Scripts\activate
pip3 install -r requirements.txt  # Use pip3 on macOS
uvicorn app.main:app --reload
```

### Frontend Only
```bash
npm install
npm run dev
```

## 🌐 Access Points

- **Frontend:** http://localhost:3000
- **Backend API:** http://localhost:8000
- **API Docs:** http://localhost:8000/docs
- **Health Check:** http://localhost:8000/health

## 🛑 Stopping Services

Press `Ctrl+C` in the terminal running the script.

## 📚 More Information

- **Environment Setup:** See `ENV_SETUP.md`
- **Backend Details:** See `BACKEND_SETUP.md`
- **Full Guide:** See `RUN_LOCALLY.md`

