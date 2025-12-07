'use client';

import Link from 'next/link';
import { usePathname } from 'next/navigation';
import ThemeSwitcher from './ThemeSwitcher';
import { isAuthenticated, getUser, signOut } from '@/lib/auth-client';
import { useEffect, useState } from 'react';

export default function Navbar() {
  const pathname = usePathname();
  const [user, setUser] = useState<any>(null);
  const [mounted, setMounted] = useState(false);

  // Function to update auth state
  const updateAuthState = () => {
    if (isAuthenticated()) {
      setUser(getUser());
    } else {
      setUser(null);
    }
  };

  // Mount-only: set up listeners and initial auth state
  useEffect(() => {
    setMounted(true);
    updateAuthState();

    // Listen for storage changes (when user signs in/out in another tab)
    const handleStorageChange = (e: StorageEvent) => {
      if (e.key === 'auth_token' || e.key === 'user') {
        updateAuthState();
      }
    };

    // Listen for custom events (for same-tab updates after login/logout)
    const handleCustomStorageChange = () => {
      updateAuthState();
    };

    window.addEventListener('storage', handleStorageChange);
    window.addEventListener('auth-state-changed', handleCustomStorageChange);

    return () => {
      window.removeEventListener('storage', handleStorageChange);
      window.removeEventListener('auth-state-changed', handleCustomStorageChange);
    };
  }, []);

  // Re-check auth state when route changes (e.g. after signin redirect)
  useEffect(() => {
    if (!mounted) return;
    updateAuthState();
  }, [pathname, mounted]);

  const isActive = (path: string) => pathname === path;

  return (
    <nav className="sticky top-0 z-50 bg-white/95 dark:bg-slate-900/95 backdrop-blur-md border-b border-slate-200 dark:border-slate-800 shadow-sm">
      <div className="container mx-auto px-4">
        <div className="flex items-center justify-between h-16">
          {/* Logo */}
          <Link
            href="/"
            className="flex items-center space-x-2 text-xl font-bold text-slate-900 dark:text-white hover:text-indigo-700 dark:hover:text-indigo-400 transition-colors"
          >
            <span className="font-semibold tracking-tight">Physical AI</span>
          </Link>

          {/* Navigation Links */}
          <div className="hidden md:flex items-center space-x-6">
            <Link
              href="/intro"
              className={`px-3 py-2 rounded-md text-sm font-medium transition-colors ${
                isActive('/intro')
                  ? 'text-indigo-700 dark:text-indigo-400 bg-indigo-50 dark:bg-indigo-900/20'
                  : 'text-slate-700 dark:text-slate-300 hover:text-indigo-700 dark:hover:text-indigo-400'
              }`}
            >
              Introduction
            </Link>
            <Link
              href="/module1"
              className={`px-3 py-2 rounded-md text-sm font-medium transition-colors ${
                isActive('/module1') || pathname?.startsWith('/module1')
                  ? 'text-indigo-700 dark:text-indigo-400 bg-indigo-50 dark:bg-indigo-900/20'
                  : 'text-slate-700 dark:text-slate-300 hover:text-indigo-700 dark:hover:text-indigo-400'
              }`}
            >
              Modules
            </Link>
            <Link
              href="/capstone"
              className={`px-3 py-2 rounded-md text-sm font-medium transition-colors ${
                isActive('/capstone') || pathname?.startsWith('/capstone')
                  ? 'text-indigo-700 dark:text-indigo-400 bg-indigo-50 dark:bg-indigo-900/20'
                  : 'text-slate-700 dark:text-slate-300 hover:text-indigo-700 dark:hover:text-indigo-400'
              }`}
            >
              Capstone
            </Link>
          </div>

          {/* Right Side */}
          <div className="flex items-center space-x-4">
            {mounted && user ? (
              <div className="flex items-center space-x-3">
                <span className="text-sm text-slate-600 dark:text-slate-400 hidden sm:inline">
                  {user.name || user.email}
                </span>
                <button
                  onClick={() => {
                    signOut();
                    setUser(null);
                  }}
                  className="text-sm text-slate-600 dark:text-slate-400 hover:text-slate-900 dark:hover:text-slate-200"
                >
                  Sign Out
                </button>
              </div>
            ) : (
              <div className="flex items-center space-x-2">
                <Link
                  href="/signin"
                  className="text-sm text-slate-600 dark:text-slate-400 hover:text-indigo-700 dark:hover:text-indigo-400"
                >
                  Sign In
                </Link>
                <span className="text-slate-400">|</span>
                <Link
                  href="/signup"
                  className="text-sm text-slate-600 dark:text-slate-400 hover:text-indigo-700 dark:hover:text-indigo-400"
                >
                  Sign Up
                </Link>
              </div>
            )}
            <ThemeSwitcher />
          </div>
        </div>
      </div>
    </nav>
  );
}

