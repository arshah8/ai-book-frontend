# Quick Start Guide

## 🚀 3-Hour Hackathon Setup

### Step 1: Install Dependencies (2 minutes)

```bash
cd ai-book
npm install
```

### Step 2: Configure Environment (1 minute)

Create `.env.local` file:
```bash
OPENAI_API_KEY=sk-your-key-here
```

Get your OpenAI API key from: https://platform.openai.com/api-keys

### Step 3: Run Development Server (1 minute)

```bash
npm run dev
```

Open http://localhost:3000

### Step 4: Test the Chatbot

1. Navigate to any module page
2. Select some text from the page
3. Click the chatbot icon (bottom right)
4. Ask a question about the selected text or general questions

### Step 5: Deploy to Vercel (5 minutes)

1. Push to GitHub:
```bash
git init
git add .
git commit -m "Initial commit"
git remote add origin <your-repo-url>
git push -u origin main
```

2. Deploy on Vercel:
   - Go to https://vercel.com
   - Import your GitHub repo
   - Add `OPENAI_API_KEY` in environment variables
   - Deploy!

## ✅ Features Implemented

- ✅ Next.js 15 with TypeScript
- ✅ Complete course content (4 modules + capstone)
- ✅ RAG chatbot with OpenAI
- ✅ Text selection support
- ✅ Modern UI with Tailwind CSS
- ✅ Responsive design
- ✅ Ready for Vercel deployment

## 📝 Project Structure

```
ai-book/
├── app/
│   ├── api/chat/          # RAG chatbot API endpoint
│   ├── intro/             # Introduction page
│   ├── module1-4/         # Course modules
│   └── capstone/          # Capstone project
├── components/
│   └── Chatbot.tsx        # Chatbot UI component
└── public/                # Static assets
```

## 🎯 Next Steps (Optional Enhancements)

For bonus points, you can add:
- Better-auth for signup/signin
- User background questions
- Content personalization
- Urdu translation
- Qdrant + Neon Postgres integration
- FastAPI backend

## 📚 Course Content

All course modules are complete with:
- Module 1: ROS 2 Fundamentals
- Module 2: Gazebo & Unity Simulation
- Module 3: NVIDIA Isaac Platform
- Module 4: Vision-Language-Action
- Capstone: Autonomous Humanoid Project

## 🐛 Troubleshooting

**Chatbot not working?**
- Check that OPENAI_API_KEY is set correctly
- Verify API key has credits
- Check browser console for errors

**Build errors?**
- Run `npm install` again
- Delete `node_modules` and `.next` folder
- Reinstall: `rm -rf node_modules .next && npm install`

## 📞 Support

For issues, check the README.md or create an issue in the repository.

