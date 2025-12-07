'use client';

import { ReactNode, useState, useEffect } from 'react';
import { usePathname } from 'next/navigation';
import Sidebar from './Sidebar';
import MobileSidebarToggle from './MobileSidebarToggle';

interface DocsLayoutProps {
  children: ReactNode;
}

export default function DocsLayout({ children }: DocsLayoutProps) {
  const pathname = usePathname();
  const [isMobileOpen, setIsMobileOpen] = useState(false);
  // Default to mobile to avoid initial horizontal scroll on small screens
  const [isMobile, setIsMobile] = useState(true);

  useEffect(() => {
    const checkMobile = () => {
      const mobile = window.innerWidth < 768;
      setIsMobile(mobile);
      if (!mobile) {
        setIsMobileOpen(false);
      }
    };
    checkMobile();
    window.addEventListener('resize', checkMobile);
    return () => window.removeEventListener('resize', checkMobile);
  }, []);

  // Close mobile sidebar when route changes
  useEffect(() => {
    setIsMobileOpen(false);
  }, [pathname]);

  return (
    <div className="flex min-h-screen overflow-x-hidden">
      <MobileSidebarToggle
        onToggle={() => setIsMobileOpen(!isMobileOpen)}
        isOpen={isMobileOpen}
      />
      <Sidebar
        isMobileOpen={isMobileOpen}
        onMobileToggle={() => setIsMobileOpen(!isMobileOpen)}
      />
      <main
        className={`flex-1 transition-all duration-300 overflow-x-hidden ${
          isMobile ? 'ml-0' : 'ml-64'
        }`}
      >
        {children}
      </main>
    </div>
  );
}

