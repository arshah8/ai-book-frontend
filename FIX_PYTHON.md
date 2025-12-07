# Fixing Python Issues on macOS

## Problem: `python: command not found`

On macOS, Python 3 is typically installed as `python3`, not `python`.

## Quick Fix

### Option 1: Use Python Setup Script (Recommended)

```bash
cd backend
./setup.sh
```

This script will:
- Detect if `python3` or `python` is available
- Create virtual environment
- Install all dependencies

### Option 2: Manual Fix

1. **Check if Python 3 is installed:**
   ```bash
   python3 --version
   ```

2. **If Python 3 is installed, use it:**
   ```bash
   cd backend
   python3 -m venv venv
   source venv/bin/activate
   pip3 install -r requirements.txt
   ```

3. **If Python 3 is NOT installed:**
   
   **Install via Homebrew (Recommended):**
   ```bash
   brew install python3
   ```
   
   **Or download from:**
   - https://www.python.org/downloads/
   - Choose Python 3.10 or higher

## After Setup

Once Python is set up, you can:

1. **Start backend:**
   ```bash
   cd backend
   source venv/bin/activate
   uvicorn app.main:app --reload
   ```

2. **Or use the run script:**
   ```bash
   ./run-dev.sh
   ```

## Verify Installation

```bash
python3 --version  # Should show Python 3.10+
pip3 --version     # Should show pip version
```

## Common Issues

### "command not found: python3"
- Install Python 3 from python.org or using Homebrew
- Make sure it's added to your PATH

### "No module named venv"
- Your Python installation might be incomplete
- Reinstall Python 3

### "pip: command not found"
- Use `pip3` instead of `pip`
- Or install pip: `python3 -m ensurepip --upgrade`

