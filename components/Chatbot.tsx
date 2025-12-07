'use client';

import { useState, useRef, useEffect } from 'react';
import { getAuthToken } from '@/lib/auth-client';
import Link from 'next/link';

interface Message {
  role: 'user' | 'assistant';
  content: string;
}

export default function Chatbot() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [selectedText, setSelectedText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    // Check authentication on mount
    const token = getAuthToken();
    setIsAuthenticated(!!token);

    const handleSelection = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().trim()) {
        setSelectedText(selection.toString().trim());
      } else {
        setSelectedText('');
      }
    };

    document.addEventListener('selectionchange', handleSelection);
    
    // Listen for storage changes (e.g. login/logout in other tabs or components)
    const handleStorageChange = () => {
        const token = getAuthToken();
        setIsAuthenticated(!!token);
    };
    window.addEventListener('storage', handleStorageChange);

    return () => {
        document.removeEventListener('selectionchange', handleSelection);
        window.removeEventListener('storage', handleStorageChange);
    };
  }, []);

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const handleSend = async () => {
    if (!input.trim() && !selectedText) return;

    if (!isAuthenticated) {
        setMessages(prev => [...prev, { 
            role: 'assistant', 
            content: 'Please sign in to use the AI assistant.' 
        }]);
        return;
    }

    const userMessage = selectedText 
      ? `${input}\n\n[Selected text: ${selectedText}]`
      : input;

    setMessages(prev => [...prev, { role: 'user', content: userMessage }]);
    setInput('');
    setSelectedText('');
    setIsLoading(true);

    try {
      const token = getAuthToken();
      const headers: HeadersInit = { 'Content-Type': 'application/json' };
      
      // Add auth token if user is logged in
      if (token) {
        headers['Authorization'] = `Bearer ${token}`;
      }

      const response = await fetch('/api/chat', {
        method: 'POST',
        headers,
        body: JSON.stringify({
          message: input || selectedText,
          context: selectedText || undefined,
        }),
      });

      const data = await response.json();
      setMessages(prev => [...prev, { role: 'assistant', content: data.response }]);
    } catch (error) {
      setMessages(prev => [...prev, { 
        role: 'assistant', 
        content: 'Sorry, I encountered an error. Please try again.' 
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <>
      <button
        onClick={() => setIsOpen(!isOpen)}
        className="fixed bottom-6 right-6 bg-indigo-600 text-white rounded-full w-16 h-16 shadow-lg hover:bg-indigo-700 transition-colors flex items-center justify-center z-50"
        aria-label="Toggle chatbot"
      >
        <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M8 12h.01M12 12h.01M16 12h.01M21 12c0 4.418-4.03 8-9 8a9.863 9.863 0 01-4.255-.949L3 20l1.395-3.72C3.512 15.042 3 13.574 3 12c0-4.418 4.03-8 9-8s9 3.582 9 8z" />
        </svg>
      </button>

      {isOpen && (
        <div className="fixed bottom-24 right-6 w-96 h-[600px] bg-white dark:bg-gray-800 rounded-lg shadow-2xl flex flex-col z-50 border border-gray-200 dark:border-gray-700">
          <div className="bg-indigo-600 text-white p-4 rounded-t-lg flex justify-between items-center">
            <h3 className="font-semibold">AI Assistant</h3>
            <button
              onClick={() => setIsOpen(false)}
              className="text-white hover:text-gray-200"
            >
              <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
              </svg>
            </button>
          </div>

          {selectedText && isAuthenticated && (
            <div className="bg-yellow-50 dark:bg-yellow-900/20 p-2 text-sm border-b border-yellow-200 dark:border-yellow-800">
              <p className="text-xs text-gray-600 dark:text-gray-400 mb-1">Selected text:</p>
              <p className="text-gray-800 dark:text-gray-200 truncate">{selectedText}</p>
            </div>
          )}

          <div className="flex-1 overflow-y-auto p-4 space-y-4">
            {/* Show login prompt if not authenticated */}
            {!isAuthenticated && (
                <div className="flex flex-col items-center justify-center h-full text-center space-y-4">
                    <div className="bg-indigo-50 dark:bg-indigo-900/20 p-6 rounded-lg">
                        <svg className="w-12 h-12 text-indigo-600 dark:text-indigo-400 mx-auto mb-3" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 15v2m-6 4h12a2 2 0 002-2v-6a2 2 0 00-2-2H6a2 2 0 00-2 2v6a2 2 0 002 2zm10-10V7a4 4 0 00-8 0v4h8z" />
                        </svg>
                        <h4 className="text-lg font-semibold text-gray-900 dark:text-white mb-2">Sign in to Chat</h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mb-4">
                            Unlock the AI assistant to ask questions and get personalized help with the course material.
                        </p>
                        <div className="flex flex-col gap-2">
                            <Link 
                                href="/signin" 
                                className="bg-indigo-600 text-white px-4 py-2 rounded-lg hover:bg-indigo-700 transition-colors w-full"
                            >
                                Sign In
                            </Link>
                            <Link 
                                href="/signup" 
                                className="text-indigo-600 dark:text-indigo-400 text-sm hover:underline"
                            >
                                Create an account
                            </Link>
                        </div>
                    </div>
                </div>
            )}

            {/* Show chat messages if authenticated */}
            {isAuthenticated && messages.length === 0 && (
              <div className="text-center text-gray-500 dark:text-gray-400 mt-8">
                <p>Ask me anything about Physical AI & Humanoid Robotics!</p>
                <p className="text-sm mt-2">Select text from the page to ask questions about specific content.</p>
              </div>
            )}
            
            {isAuthenticated && messages.map((msg, idx) => (
              <div
                key={idx}
                className={`flex ${msg.role === 'user' ? 'justify-end' : 'justify-start'}`}
              >
                <div
                  className={`max-w-[80%] rounded-lg p-3 ${
                    msg.role === 'user'
                      ? 'bg-indigo-600 text-white'
                      : 'bg-gray-100 dark:bg-gray-700 text-gray-900 dark:text-gray-100'
                  }`}
                >
                  <p className="whitespace-pre-wrap">{msg.content}</p>
                </div>
              </div>
            ))}
            
            {isLoading && (
              <div className="flex justify-start">
                <div className="bg-gray-100 dark:bg-gray-700 rounded-lg p-3">
                  <div className="flex space-x-2">
                    <div className="w-2 h-2 bg-gray-400 rounded-full animate-bounce"></div>
                    <div className="w-2 h-2 bg-gray-400 rounded-full animate-bounce" style={{ animationDelay: '0.2s' }}></div>
                    <div className="w-2 h-2 bg-gray-400 rounded-full animate-bounce" style={{ animationDelay: '0.4s' }}></div>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Only show input area if authenticated */}
          {isAuthenticated && (
            <div className="border-t border-gray-200 dark:border-gray-700 p-4">
                <div className="flex gap-2">
                <input
                    type="text"
                    value={input}
                    onChange={(e) => setInput(e.target.value)}
                    onKeyPress={(e) => e.key === 'Enter' && !e.shiftKey && handleSend()}
                    placeholder={selectedText ? "Ask about selected text..." : "Type your question..."}
                    className="flex-1 px-4 py-2 border border-gray-300 dark:border-gray-600 rounded-lg bg-white dark:bg-gray-700 text-gray-900 dark:text-gray-100 focus:outline-none focus:ring-2 focus:ring-indigo-500"
                />
                <button
                    onClick={handleSend}
                    disabled={isLoading || (!input.trim() && !selectedText)}
                    className="bg-indigo-600 text-white px-4 py-2 rounded-lg hover:bg-indigo-700 disabled:opacity-50 disabled:cursor-not-allowed transition-colors"
                >
                    Send
                </button>
                </div>
            </div>
          )}
        </div>
      )}
    </>
  );
}
