import { db, translations } from "./database";
import { eq, and } from "drizzle-orm";
import { v4 as uuidv4 } from "uuid";

const BACKEND_URL = process.env.BACKEND_URL || process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000';

export async function translateContent(
  text: string,
  language: "ur" | "en",
  module?: string
): Promise<string> {
  // Check cache first
  try {
    const cached = await db
      .select()
      .from(translations)
      .where(
        and(
          eq(translations.originalText, text),
          eq(translations.language, language)
        )
      )
      .limit(1);

    if (cached.length > 0) {
      return cached[0].translatedText;
    }
  } catch (error) {
    console.error("Error checking translation cache:", error);
  }

  // Translate using backend API
  try {
    const response = await fetch(`${BACKEND_URL}/api/translate`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ text, language: language || 'ur', module }),
    });

    if (!response.ok) {
      throw new Error('Translation API request failed');
    }

    const data = await response.json();
    const translated = data.translated_text || data.text || text;

    // Cache the translation
    try {
      await db.insert(translations).values({
        id: uuidv4(),
        originalText: text,
        translatedText: translated,
        language,
        module: module || null,
      });
    } catch (error) {
      console.error("Error caching translation:", error);
      // Continue even if caching fails
    }

    return translated;
  } catch (error) {
    console.error("Error translating content:", error);
    throw new Error("Failed to translate content");
  }
}

// Extract text content from HTML/JSX (simple version)
export function extractTextContent(html: string): string {
  // Remove HTML tags and extract text
  return html
    .replace(/<[^>]*>/g, " ")
    .replace(/\s+/g, " ")
    .trim();
}

