#!/bin/bash
# Script to set up environment files

echo "Setting up environment files..."

# Frontend .env.local
if [ ! -f .env.local ]; then
  cp env.local.example .env.local
  echo "✓ Created .env.local (frontend)"
  echo "  Please edit .env.local and add your BACKEND_URL"
else
  echo "⚠ .env.local already exists, skipping"
fi

# Backend .env
if [ ! -f backend/.env ]; then
  cp backend/env.example backend/.env
  echo "✓ Created backend/.env"
  echo "  Please edit backend/.env and add your API keys:"
  echo "    - OPENAI_API_KEY"
  echo "    - QDRANT_URL and QDRANT_API_KEY"
  echo "    - DATABASE_URL"
  echo "    - BETTER_AUTH_SECRET"
else
  echo "⚠ backend/.env already exists, skipping"
fi

echo ""
echo "Next steps:"
echo "1. Edit .env.local and add BACKEND_URL"
echo "2. Edit backend/.env and add all API keys"
echo "3. See ENV_SETUP.md for detailed instructions"
