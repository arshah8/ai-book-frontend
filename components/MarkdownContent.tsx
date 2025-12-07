'use client';

import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import rehypeHighlight from 'rehype-highlight';
import rehypeRaw from 'rehype-raw';
import 'highlight.js/styles/github.css';

interface MarkdownContentProps {
  content: string;
  className?: string;
}

export default function MarkdownContent({ content, className = '' }: MarkdownContentProps) {
  return (
    <article className={`prose prose-lg dark:prose-invert max-w-none ${className}`}>
      <ReactMarkdown
        remarkPlugins={[remarkGfm]}
        rehypePlugins={[rehypeRaw, rehypeHighlight]}
        components={{
          code({ node, inline, className, children, ...props }: any) {
            const match = /language-(\w+)/.exec(className || '');
            return !inline && match ? (
              <pre className="!bg-slate-50 dark:!bg-slate-900 !p-4 !rounded-lg !overflow-x-auto !border !border-slate-200 dark:!border-slate-800 !text-slate-900 dark:!text-slate-100">
                <code className={className} {...props}>
                  {children}
                </code>
              </pre>
            ) : (
              <code className="!bg-slate-100 dark:!bg-slate-800 !text-indigo-700 dark:!text-indigo-300 !px-1.5 !py-0.5 !rounded !text-sm !font-mono !border !border-slate-200 dark:!border-slate-700" {...props}>
                {children}
              </code>
            );
          },
          h1: ({ children }) => (
            <h1 className="!text-3xl !font-bold !mb-6 !mt-0 !text-slate-900 dark:!text-slate-100">
              {children}
            </h1>
          ),
          h2: ({ children }) => (
            <h2 className="!text-2xl !font-semibold !mb-4 !mt-8 !text-slate-900 dark:!text-slate-100 !border-b !border-slate-200 dark:!border-slate-800 !pb-2">
              {children}
            </h2>
          ),
          h3: ({ children }) => (
            <h3 className="!text-xl !font-semibold !mb-3 !mt-6 !text-slate-800 dark:!text-slate-200">
              {children}
            </h3>
          ),
          p: ({ children }) => (
            <p className="!mb-4 !leading-7 !text-slate-700 dark:!text-slate-300">
              {children}
            </p>
          ),
          a: ({ href, children }) => (
            <a
              href={href}
              className="!text-indigo-700 dark:!text-indigo-400 !font-medium hover:!text-indigo-900 dark:hover:!text-indigo-300 hover:!underline !transition-colors"
            >
              {children}
            </a>
          ),
        }}
      >
        {content}
      </ReactMarkdown>
    </article>
  );
}

