#!/bin/bash

# Script to run both frontend and backend locally
# Usage: ./run-dev.sh

set -e

echo "🚀 Starting Physical AI Textbook Development Environment"
echo ""

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if .env files exist
if [ ! -f ".env.local" ]; then
    echo -e "${YELLOW}⚠️  .env.local not found. Creating from example...${NC}"
    if [ -f "env.local.example" ]; then
        cp env.local.example .env.local
        echo -e "${GREEN}✓ Created .env.local${NC}"
        echo -e "${YELLOW}  Please edit .env.local and set BACKEND_URL${NC}"
    else
        echo -e "${YELLOW}⚠️  env.local.example not found${NC}"
    fi
fi

if [ ! -f "backend/.env" ]; then
    echo -e "${YELLOW}⚠️  backend/.env not found. Creating from example...${NC}"
    if [ -f "backend/env.example" ]; then
        cp backend/env.example backend/.env
        echo -e "${GREEN}✓ Created backend/.env${NC}"
        echo -e "${YELLOW}  Please edit backend/.env and add your API keys${NC}"
    else
        echo -e "${YELLOW}⚠️  backend/env.example not found${NC}"
    fi
fi

# Check if Python virtual environment exists
if [ ! -d "backend/venv" ]; then
    echo -e "${YELLOW}⚠️  Python virtual environment not found. Creating...${NC}"
    cd backend
    python3 -m venv venv || python -m venv venv
    echo -e "${GREEN}✓ Created virtual environment${NC}"
    echo -e "${BLUE}Installing Python dependencies...${NC}"
    source venv/bin/activate
    pip install -r requirements.txt
    cd ..
    echo -e "${GREEN}✓ Python dependencies installed${NC}"
fi

# Check if node_modules exists
if [ ! -d "node_modules" ]; then
    echo -e "${YELLOW}⚠️  Node modules not found. Installing...${NC}"
    npm install
    echo -e "${GREEN}✓ Node dependencies installed${NC}"
fi

echo ""
echo -e "${BLUE}Starting Backend (FastAPI)...${NC}"
echo ""

# Start backend in background
cd backend
source venv/bin/activate
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000 &
BACKEND_PID=$!
cd ..

# Wait a moment for backend to start
sleep 3

echo ""
echo -e "${BLUE}Starting Frontend (Next.js)...${NC}"
echo ""

# Start frontend
npm run dev &
FRONTEND_PID=$!

echo ""
echo -e "${GREEN}✓ Both services are starting!${NC}"
echo ""
echo -e "${GREEN}Backend:${NC}  http://localhost:8000"
echo -e "${GREEN}Frontend:${NC} http://localhost:3000"
echo -e "${GREEN}API Docs:${NC} http://localhost:8000/docs"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop both services${NC}"
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo -e "${YELLOW}Stopping services...${NC}"
    kill $BACKEND_PID 2>/dev/null || true
    kill $FRONTEND_PID 2>/dev/null || true
    echo -e "${GREEN}✓ Services stopped${NC}"
    exit 0
}

# Trap Ctrl+C
trap cleanup INT TERM

# Wait for both processes
wait

