import Link from "next/link";
import Chatbot from "@/components/Chatbot";
import DocsLayout from '@/components/DocsLayout';
import MarkdownContent from '@/components/MarkdownContent';
import { loadMarkdownContent } from '@/lib/markdown-loader';

export default async function IntroPage() {
  // Load introduction content from Docusaurus
  const content = await loadMarkdownContent('intro.md');
  
  return (
    <DocsLayout>
      <div className="min-h-screen bg-white dark:bg-slate-900 transition-colors">
        <div className="max-w-4xl mx-auto px-4 md:px-8 py-8 md:py-12">

        {/* Render Docusaurus content */}
        {content ? (
          <MarkdownContent content={content} />
        ) : (
          <article className="prose prose-lg dark:prose-invert max-w-none">
            <h1>Introduction</h1>
            <p>Welcome to Physical AI & Humanoid Robotics!</p>
          </article>
        )}

        <div className="mt-12 pt-8 border-t border-gray-200 dark:border-slate-800 flex gap-4 justify-end">
          <Link
            href="/module1"
            className="text-sm text-gray-600 dark:text-gray-400 hover:text-indigo-600 dark:hover:text-indigo-400 font-medium"
          >
            Start Module 1 →
          </Link>
        </div>
        </div>
      </div>
      <Chatbot />
    </DocsLayout>
  );
}
