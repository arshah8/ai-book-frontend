import { drizzle } from 'drizzle-orm/neon-http';
import { neon } from '@neondatabase/serverless';
import { pgTable, text, timestamp, varchar, pgEnum } from 'drizzle-orm/pg-core';

// Database connection - lazy initialization to avoid build-time errors
// During Vercel build, DATABASE_URL is not available, so we defer initialization
let _db: ReturnType<typeof drizzle> | null = null;

function initDb() {
  if (!_db) {
    const databaseUrl = process.env.DATABASE_URL;
    if (!databaseUrl || databaseUrl === '') {
      throw new Error('DATABASE_URL environment variable is required. Database operations should go through the backend API.');
    }
    const sql = neon(databaseUrl);
    // Type assertion to fix compatibility between neon and drizzle types
    _db = drizzle(sql as any);
  }
  return _db;
}

// Export db with getter that initializes on first access
// This allows the module to load during build without DATABASE_URL
export const db = new Proxy({} as ReturnType<typeof drizzle>, {
  get(_target, prop) {
    return initDb()[prop as keyof ReturnType<typeof drizzle>];
  }
});

// Enums
export const experienceLevelEnum = pgEnum('experience_level', ['beginner', 'intermediate', 'advanced']);
export const languageEnum = pgEnum('preferred_language', ['en', 'ur']);

// User profiles table
export const userProfiles = pgTable('user_profiles', {
  id: text('id').primaryKey(),
  userId: text('user_id').notNull(),
  softwareBackground: text('software_background'),
  hardwareBackground: text('hardware_background'),
  experienceLevel: experienceLevelEnum('experience_level').default('beginner'),
  preferredLanguage: languageEnum('preferred_language').default('en'),
  createdAt: timestamp('created_at').defaultNow(),
  updatedAt: timestamp('updated_at').defaultNow(),
});

// Chat history table
export const chatHistory = pgTable('chat_history', {
  id: text('id').primaryKey(),
  userId: text('user_id').notNull(),
  message: text('message').notNull(),
  response: text('response').notNull(),
  context: text('context'),
  timestamp: timestamp('timestamp').defaultNow(),
});

// Content chunks table (for RAG)
export const contentChunks = pgTable('content_chunks', {
  id: text('id').primaryKey(),
  content: text('content').notNull(),
  module: varchar('module', { length: 50 }),
  section: varchar('section', { length: 100 }),
  embeddingId: text('embedding_id'), // Qdrant vector ID
  createdAt: timestamp('created_at').defaultNow(),
});

// Translation cache table
export const translations = pgTable('translations', {
  id: text('id').primaryKey(),
  originalText: text('original_text').notNull(),
  translatedText: text('translated_text').notNull(),
  language: varchar('language', { length: 10 }).notNull(),
  module: varchar('module', { length: 50 }),
  createdAt: timestamp('created_at').defaultNow(),
});

export type UserProfile = typeof userProfiles.$inferSelect;
export type ChatHistory = typeof chatHistory.$inferSelect;
export type ContentChunk = typeof contentChunks.$inferSelect;
export type Translation = typeof translations.$inferSelect;

