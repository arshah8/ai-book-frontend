import { notFound } from 'next/navigation';
import Link from 'next/link';
import Chatbot from '@/components/Chatbot';
import MarkdownContent from '@/components/MarkdownContent';
import DocsLayout from '@/components/DocsLayout';
import { loadMarkdownContent, getModuleFiles } from '@/lib/markdown-loader';

interface PageProps {
  params: Promise<{
    section: string;
  }>;
}

export async function generateStaticParams() {
  const sections = await getModuleFiles('module1');
  return sections.map((section) => ({
    section,
  }));
}

export default async function Module1SectionPage({ params }: PageProps) {
  const { section } = await params;
  
  // Load markdown content from Docusaurus
  const content = await loadMarkdownContent(`module1/${section}.md`);
  
  if (!content) {
    notFound();
  }
  
  // Get section title from content
  const titleMatch = content.match(/^#\s+(.+)$/m);
  const title = titleMatch ? titleMatch[1] : `Module 1: ${section}`;
  
  // Get all sections for navigation
  const allSections = await getModuleFiles('module1');
  const currentIndex = allSections.indexOf(section);
  const prevSection = currentIndex > 0 ? allSections[currentIndex - 1] : null;
  const nextSection = currentIndex < allSections.length - 1 ? allSections[currentIndex + 1] : null;
  
  return (
    <DocsLayout>
      <div className="min-h-screen bg-white dark:bg-slate-900 transition-colors">
        <div className="max-w-4xl mx-auto px-4 md:px-8 py-8 md:py-12">
        <MarkdownContent content={content} />

        <div className="mt-12 pt-8 border-t border-gray-200 dark:border-slate-800 flex gap-4 justify-between">
          <div>
            {prevSection && (
              <Link
                href={`/module1/${prevSection}`}
                className="text-sm text-gray-600 dark:text-gray-400 hover:text-indigo-600 dark:hover:text-indigo-400 font-medium"
              >
                ← Previous
              </Link>
            )}
          </div>
          <div>
            {nextSection ? (
              <Link
                href={`/module1/${nextSection}`}
                className="text-sm text-gray-600 dark:text-gray-400 hover:text-indigo-600 dark:hover:text-indigo-400 font-medium"
              >
                Next →
              </Link>
            ) : (
              <Link
                href="/module2"
                className="text-sm text-gray-600 dark:text-gray-400 hover:text-indigo-600 dark:hover:text-indigo-400 font-medium"
              >
                Next: Module 2 →
              </Link>
            )}
          </div>
          </div>
        </div>
      </div>
      <Chatbot />
    </DocsLayout>
  );
}
