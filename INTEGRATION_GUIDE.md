# Docusaurus Content Integration Guide

## Overview

Your Next.js app now integrates Docusaurus markdown content while keeping all your custom components (like Chatbot).

## How It Works

1. **Markdown Files**: Docusaurus content is stored in `book/docs/`
2. **Markdown Loader**: `lib/markdown-loader.ts` reads markdown files at build time
3. **Markdown Renderer**: `components/MarkdownContent.tsx` renders markdown with syntax highlighting
4. **Next.js Pages**: Your existing pages now load and display Docusaurus content

## Structure

```
app/
├── module1/
│   ├── page.tsx              # Module overview (shows TOC from Docusaurus)
│   └── [section]/
│       └── page.tsx          # Individual sections (loads from Docusaurus)
├── intro/
│   └── page.tsx              # Introduction (loads from Docusaurus)
└── ...

book/docs/                    # Docusaurus content source
├── intro.md
├── module1/
│   ├── introduction.md
│   ├── architecture.md
│   └── ...
```

## Features

✅ **Custom Components**: Your Chatbot and other components still work  
✅ **Docusaurus Content**: All content comes from Docusaurus markdown  
✅ **Syntax Highlighting**: Code blocks are highlighted  
✅ **Navigation**: Automatic TOC generation from Docusaurus structure  
✅ **Dark Mode**: Supports your existing dark mode  

## Adding New Content

1. Add markdown file to `book/docs/moduleX/section.md`
2. The page will automatically appear in the TOC
3. Access via `/moduleX/section`

## Benefits

- **Single Source of Truth**: Content in Docusaurus markdown
- **Easy Updates**: Edit markdown, content updates automatically
- **Custom UI**: Keep your Next.js design and components
- **Best of Both**: Docusaurus content + Next.js features

