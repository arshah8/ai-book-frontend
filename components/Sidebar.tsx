'use client';

import Link from 'next/link';
import { usePathname } from 'next/navigation';
import { useState, useEffect } from 'react';
import MobileSidebarToggle from './MobileSidebarToggle';

interface SidebarSection {
  title: string;
  href: string;
  items?: { title: string; href: string }[];
}

const sidebarSections: SidebarSection[] = [
  {
    title: 'Introduction',
    href: '/intro',
  },
  {
    title: 'Module 1: The Robotic Nervous System (ROS 2)',
    href: '/module1',
    items: [
      { title: 'Introduction', href: '/module1/introduction' },
      { title: 'ROS 2 Architecture', href: '/module1/architecture' },
      { title: 'Nodes, Topics, and Services', href: '/module1/nodes-topics-services' },
      { title: 'Understanding URDF', href: '/module1/urdf' },
      { title: 'Bridging Python Agents to ROS 2', href: '/module1/python-agents' },
    ],
  },
  {
    title: 'Module 2: The Digital Twin (Gazebo & Unity)',
    href: '/module2',
    items: [
      { title: 'Introduction', href: '/module2/introduction' },
      { title: 'Gazebo Simulation', href: '/module2/gazebo-simulation' },
      { title: 'Unity Rendering', href: '/module2/unity-rendering' },
      { title: 'Sensor Simulation', href: '/module2/sensors' },
    ],
  },
  {
    title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
    href: '/module3',
    items: [
      { title: 'Introduction', href: '/module3/introduction' },
      { title: 'NVIDIA Isaac Sim', href: '/module3/isaac-sim' },
      { title: 'Isaac ROS', href: '/module3/isaac-ros' },
      { title: 'Nav2 Path Planning', href: '/module3/nav2' },
    ],
  },
  {
    title: 'Module 4: Vision-Language-Action (VLA)',
    href: '/module4',
    items: [
      { title: 'Introduction', href: '/module4/introduction' },
      { title: 'Voice-to-Action', href: '/module4/voice-to-action' },
      { title: 'Cognitive Planning', href: '/module4/cognitive-planning' },
      { title: 'LLM Integration', href: '/module4/llm-integration' },
    ],
  },
  {
    title: 'Capstone Project',
    href: '/capstone',
    items: [
      { title: 'Project Overview', href: '/capstone/overview' },
      { title: 'Implementation', href: '/capstone/implementation' },
      { title: 'Deployment', href: '/capstone/deployment' },
    ],
  },
];

interface SidebarProps {
  isMobileOpen?: boolean;
  onMobileToggle?: () => void;
}

export default function Sidebar({ isMobileOpen = false, onMobileToggle }: SidebarProps = {}) {
  const pathname = usePathname();
  const [expandedSections, setExpandedSections] = useState<Set<string>>(
    new Set(sidebarSections.map((s) => s.href))
  );
  const [isMobile, setIsMobile] = useState(false);
  const [internalMobileOpen, setInternalMobileOpen] = useState(false);
  
  // Use external state if provided, otherwise use internal state
  const mobileOpen = onMobileToggle !== undefined ? isMobileOpen : internalMobileOpen;
  const setMobileOpen = onMobileToggle || setInternalMobileOpen;

  useEffect(() => {
    const checkMobile = () => {
      setIsMobile(window.innerWidth < 768);
    };
    checkMobile();
    window.addEventListener('resize', checkMobile);
    return () => window.removeEventListener('resize', checkMobile);
  }, []);

  // Auto-expand section containing current page
  useEffect(() => {
    if (pathname) {
      sidebarSections.forEach((section) => {
        if (section.items) {
          const hasActiveItem = section.items.some((item) => isActive(item.href));
          if (hasActiveItem && !expandedSections.has(section.href)) {
            setExpandedSections((prev) => new Set([...prev, section.href]));
          }
        }
      });
    }
  }, [pathname]);

  const toggleSection = (href: string) => {
    const newExpanded = new Set(expandedSections);
    if (newExpanded.has(href)) {
      newExpanded.delete(href);
    } else {
      newExpanded.add(href);
    }
    setExpandedSections(newExpanded);
  };

  const isActive = (href: string) => {
    if (href === pathname) return true;
    if (pathname?.startsWith(href) && href !== '/') return true;
    return false;
  };

  return (
    <>
      {/* Mobile overlay */}
      {isMobile && mobileOpen && (
        <div
          className="fixed inset-0 bg-black/50 z-40 md:hidden"
          onClick={() => setMobileOpen(false)}
        />
      )}

      <aside
        className={`fixed left-0 top-16 h-[calc(100vh-4rem)] w-64 bg-white dark:bg-slate-900 border-r border-gray-200 dark:border-slate-800 overflow-y-auto z-40 transition-transform duration-300 ease-in-out ${
          isMobile && !mobileOpen ? '-translate-x-full' : 'translate-x-0'
        }`}
      >
        <nav className="p-4 space-y-1">
        {sidebarSections.map((section) => {
          const hasItems = section.items && section.items.length > 0;
          const isExpanded = expandedSections.has(section.href);
          const sectionActive = isActive(section.href);

          return (
            <div key={section.href} className="space-y-1">
              {hasItems ? (
                <>
                  <button
                    onClick={() => toggleSection(section.href)}
                    className={`w-full flex items-center justify-between px-3 py-2 text-sm font-medium rounded-md transition-colors ${
                      sectionActive
                        ? 'bg-indigo-50 dark:bg-indigo-900/20 text-indigo-600 dark:text-indigo-400'
                        : 'text-gray-700 dark:text-gray-300 hover:bg-gray-50 dark:hover:bg-slate-800'
                    }`}
                  >
                    <Link
                      href={section.href}
                      className="flex-1 text-left"
                      onClick={(e) => e.stopPropagation()}
                    >
                      {section.title}
                    </Link>
                    <svg
                      className={`w-4 h-4 transition-transform ${
                        isExpanded ? 'rotate-90' : ''
                      }`}
                      fill="none"
                      stroke="currentColor"
                      viewBox="0 0 24 24"
                    >
                      <path
                        strokeLinecap="round"
                        strokeLinejoin="round"
                        strokeWidth={2}
                        d="M9 5l7 7-7 7"
                      />
                    </svg>
                  </button>
                  {isExpanded && section.items && (
                    <div className="ml-4 space-y-1 border-l border-gray-200 dark:border-slate-700 pl-3">
                      {section.items.map((item) => {
                        const itemActive = isActive(item.href);
                        return (
                          <Link
                            key={item.href}
                            href={item.href}
                            className={`block px-3 py-1.5 text-sm rounded-md transition-colors ${
                              itemActive
                                ? 'bg-indigo-50 dark:bg-indigo-900/20 text-indigo-600 dark:text-indigo-400 font-medium'
                                : 'text-gray-600 dark:text-gray-400 hover:bg-gray-50 dark:hover:bg-slate-800 hover:text-gray-900 dark:hover:text-gray-200'
                            }`}
                          >
                            {item.title}
                          </Link>
                        );
                      })}
                    </div>
                  )}
                </>
              ) : (
                <Link
                  href={section.href}
                  className={`block px-3 py-2 text-sm font-medium rounded-md transition-colors ${
                    sectionActive
                      ? 'bg-indigo-50 dark:bg-indigo-900/20 text-indigo-600 dark:text-indigo-400'
                      : 'text-gray-700 dark:text-gray-300 hover:bg-gray-50 dark:hover:bg-slate-800'
                  }`}
                >
                  {section.title}
                </Link>
              )}
            </div>
          );
        })}
      </nav>
    </aside>
    </>
  );
}

