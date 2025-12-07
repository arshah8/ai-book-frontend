'use client';

import { useState, useEffect } from 'react';

interface MobileSidebarToggleProps {
  onToggle: () => void;
  isOpen: boolean;
}

export default function MobileSidebarToggle({ onToggle, isOpen }: MobileSidebarToggleProps) {
  const [isMobile, setIsMobile] = useState(false);

  useEffect(() => {
    const checkMobile = () => {
      setIsMobile(window.innerWidth < 768);
    };
    checkMobile();
    window.addEventListener('resize', checkMobile);
    return () => window.removeEventListener('resize', checkMobile);
  }, []);

  if (!isMobile) return null;

  return (
    <button
      onClick={onToggle}
      className="fixed right-4 top-20 z-50 p-2 rounded-lg bg-white dark:bg-slate-800 border border-gray-200 dark:border-slate-700 shadow-lg md:hidden"
      aria-label="Toggle sidebar"
    >
      <svg
        className="w-6 h-6 text-gray-700 dark:text-gray-300"
        fill="none"
        stroke="currentColor"
        viewBox="0 0 24 24"
      >
        {isOpen ? (
          <path
            strokeLinecap="round"
            strokeLinejoin="round"
            strokeWidth={2}
            d="M6 18L18 6M6 6l12 12"
          />
        ) : (
          <path
            strokeLinecap="round"
            strokeLinejoin="round"
            strokeWidth={2}
            d="M4 6h16M4 12h16M4 18h16"
          />
        )}
      </svg>
    </button>
  );
}

