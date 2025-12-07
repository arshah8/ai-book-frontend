import { GoogleGenerativeAI } from "@google/generative-ai";

const apiKey = process.env.GEMINI_API_KEY || "";

if (!apiKey) {
  console.warn("GEMINI_API_KEY not set. Some features may not work.");
}

export const genAI = new GoogleGenerativeAI(apiKey);

// Get text embedding using Gemini
export async function getEmbedding(text: string): Promise<number[]> {
  try {
    // Note: Gemini doesn't have a direct embedding model in the free tier
    // We'll use text-embedding-004 or fallback to a simple hash-based approach
    // For now, using a workaround with Gemini's text generation
    const model = genAI.getGenerativeModel({ model: "gemini-pro" });
    
    // Since Gemini free tier doesn't have embeddings, we'll use a simple approach
    // In production, you might want to use a separate embedding service
    // For now, we'll create a simple vector representation
    const response = await model.embedContent(text);
    
    // If Gemini provides embeddings, use them
    if (response.embedding) {
      return response.embedding.values;
    }
    
    // Fallback: create a simple hash-based embedding
    return createSimpleEmbedding(text);
  } catch (error) {
    console.error("Error getting embedding with Gemini:", error);
    // Fallback to simple embedding
    return createSimpleEmbedding(text);
  }
}

// Simple hash-based embedding as fallback (1536 dimensions to match OpenAI)
function createSimpleEmbedding(text: string): number[] {
  const embedding = new Array(1536).fill(0);
  const words = text.toLowerCase().split(/\s+/);
  
  words.forEach((word, i) => {
    let hash = 0;
    for (let j = 0; j < word.length; j++) {
      hash = ((hash << 5) - hash) + word.charCodeAt(j);
      hash = hash & hash;
    }
    const index = Math.abs(hash) % 1536;
    embedding[index] += 1 / (i + 1);
  });
  
  // Normalize
  const norm = Math.sqrt(embedding.reduce((sum, val) => sum + val * val, 0));
  return embedding.map(val => norm > 0 ? val / norm : 0);
}

// Generate chat completion using Gemini
export async function generateChatResponse(
  userMessage: string,
  systemContext?: string
): Promise<string> {
  try {
    const model = genAI.getGenerativeModel({ model: "gemini-pro" });
    
    const fullPrompt = systemContext
      ? `${systemContext}\n\nUser question: ${userMessage}\n\nAnswer based on the context provided above.`
      : userMessage;
    
    const result = await model.generateContent(fullPrompt);
    const response = await result.response;
    return response.text();
  } catch (error) {
    console.error("Error generating chat response:", error);
    throw new Error("Failed to generate response");
  }
}

// Translate text using Gemini
export async function translateText(
  text: string,
  targetLanguage: "ur" | "en" = "ur"
): Promise<string> {
  try {
    const model = genAI.getGenerativeModel({ model: "gemini-pro" });
    
    const languageName = targetLanguage === "ur" ? "Urdu" : "English";
    const prompt = `Translate the following text to ${languageName}. Preserve formatting, code blocks, and technical terms. Only return the translation:\n\n${text}`;
    
    const result = await model.generateContent(prompt);
    const response = await result.response;
    return response.text();
  } catch (error) {
    console.error("Error translating text:", error);
    throw new Error("Failed to translate text");
  }
}

