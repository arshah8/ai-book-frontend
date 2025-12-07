import Link from "next/link";
import Chatbot from "@/components/Chatbot";
import DocsLayout from '@/components/DocsLayout';
import { getModuleFiles } from '@/lib/markdown-loader';
import MarkdownContent from '@/components/MarkdownContent';
import { loadMarkdownContent } from '@/lib/markdown-loader';

export default async function Module3Page() {
  // Get all sections from Docusaurus
  const sections = await getModuleFiles('module3');
  
  // Load introduction content
  const introContent = await loadMarkdownContent('module3/introduction.md');
  
  // Map section names to display names
  const sectionMap: Record<string, string> = {
    'introduction': 'Introduction',
    'isaac-sim': 'NVIDIA Isaac Sim',
    'isaac-ros': 'Isaac ROS',
    'nav2': 'Nav2 Path Planning',
  };

  return (
    <DocsLayout>
      <div className="min-h-screen bg-white dark:bg-slate-900 transition-colors">
        <div className="max-w-4xl mx-auto px-4 md:px-8 py-8 md:py-12">

        {/* Show introduction content from Docusaurus */}
        {introContent && (
          <div className="mb-8">
            <MarkdownContent content={introContent} />
          </div>
        )}

        {/* Table of Contents from Docusaurus */}
        <div className="bg-gray-50 dark:bg-slate-800/50 rounded-lg border border-gray-200 dark:border-slate-700 p-6 mb-8">
          <h2 className="text-xl font-semibold mb-4 text-gray-900 dark:text-gray-100">Table of Contents</h2>
          <ul className="space-y-1">
            {sections.map((section) => (
              <li key={section}>
                <Link
                  href={`/module3/${section}`}
                  className="text-indigo-600 hover:text-indigo-800 dark:text-indigo-400 dark:hover:text-indigo-300 flex items-center py-1.5 hover:underline transition-colors"
                >
                  <span className="w-1.5 h-1.5 rounded-full bg-indigo-600 dark:bg-indigo-400 mr-3"></span>
                  {sectionMap[section] || section}
                </Link>
              </li>
            ))}
          </ul>
        </div>

        {/* Quick Links */}
        <div className="mt-12 pt-8 border-t border-gray-200 dark:border-slate-800 flex gap-4 justify-end">
          <Link
            href="/module4"
            className="text-sm text-gray-600 dark:text-gray-400 hover:text-indigo-600 dark:hover:text-indigo-400 font-medium"
          >
            Next: Module 4 →
          </Link>
        </div>
        </div>
      </div>
      <Chatbot />
    </DocsLayout>
  );
}
