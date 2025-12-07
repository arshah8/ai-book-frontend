# Physical AI & Humanoid Robotics Textbook

A comprehensive AI-native textbook for teaching Physical AI & Humanoid Robotics, built with Next.js and featuring an integrated RAG chatbot.

## Features

- 📚 Complete course content covering 4 modules + capstone project
- 🤖 Integrated RAG chatbot with text selection support
- 🎨 Modern, responsive UI with Tailwind CSS
- 📱 Mobile-friendly design
- 🌐 Ready for deployment on Vercel or GitHub Pages

## Getting Started

### Prerequisites

- Node.js 18+ 
- Python 3.10+
- npm or yarn
- API keys (OpenAI, Qdrant, Neon Postgres) - see ENV_SETUP.md

### Quick Start (Recommended)

1. Clone the repository:
```bash
git clone <your-repo-url>
cd ai-book
```

2. Set up environment files:
```bash
# Frontend
cp env.local.example .env.local
# Edit .env.local and set BACKEND_URL=http://localhost:8000

# Backend
cd backend
cp env.example .env
# Edit .env and add your API keys
cd ..
```

3. Run both frontend and backend:
```bash
./run-dev.sh
```

This will start:
- Backend on http://localhost:8000
- Frontend on http://localhost:3000

### Manual Setup

See `QUICK_START.md` for detailed instructions.

### Environment Variables

See `ENV_SETUP.md` for complete guide on setting up API keys.

## Project Structure

```
ai-book/
├── app/                    # Next.js app directory
│   ├── api/               # API routes
│   │   └── chat/          # RAG chatbot API
│   ├── intro/             # Introduction page
│   ├── module1-4/         # Course modules
│   └── capstone/          # Capstone project
├── components/            # React components
│   └── Chatbot.tsx        # Chatbot UI component
├── lib/                   # Utility functions
└── public/                # Static assets
```

## Deployment

### Vercel (Recommended)

1. Push your code to GitHub
2. Import your repository in [Vercel](https://vercel.com)
3. Add your environment variables
4. Deploy!

### GitHub Pages

1. Update `next.config.js` to set `output: 'export'` (already configured)
2. Build the project: `npm run build`
3. Deploy the `out` directory to GitHub Pages

## RAG Chatbot

The integrated chatbot uses:
- OpenAI embeddings for vector search
- In-memory vector store (can be upgraded to Qdrant)
- GPT-4o-mini for responses
- Text selection support for context-aware queries

## Course Content

- **Module 1:** The Robotic Nervous System (ROS 2)
- **Module 2:** The Digital Twin (Gazebo & Unity)
- **Module 3:** The AI-Robot Brain (NVIDIA Isaac™)
- **Module 4:** Vision-Language-Action (VLA)
- **Capstone:** The Autonomous Humanoid

## License

This project is part of the Panaversity initiative.

## Contributing

This is a hackathon project. For improvements, please submit issues or pull requests.

