import { translateText as geminiTranslate } from "./gemini";
import { db, translations } from "./database";
import { eq, and } from "drizzle-orm";
import { v4 as uuidv4 } from "uuid";

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

  // Translate using Gemini
  try {
    const translated = await geminiTranslate(text, language);

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

