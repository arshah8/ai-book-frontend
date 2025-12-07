import { drizzle } from 'drizzle-orm/neon-http';
import { neon } from '@neondatabase/serverless';
import { pgTable, text, timestamp, varchar, pgEnum } from 'drizzle-orm/pg-core';

// Database connection
const sql = neon(process.env.DATABASE_URL!);
// Type assertion to fix compatibility between neon and drizzle types
// This is a known issue with type compatibility between @neondatabase/serverless and drizzle-orm
export const db = drizzle(sql as any);

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

