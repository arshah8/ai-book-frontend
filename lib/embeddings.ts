// Note: This file is deprecated - embeddings are now handled by FastAPI backend
// Keeping for reference but not actively used

function generateUUID(): string {
  if (typeof crypto !== 'undefined' && crypto.randomUUID) {
    return crypto.randomUUID();
  }
  // Fallback for older environments
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, function(c) {
    const r = Math.random() * 16 | 0;
    const v = c === 'x' ? r : (r & 0x3 | 0x8);
    return v.toString(16);
  });
}

// These functions are no longer used - all embedding operations
// are handled by the FastAPI backend at /api/chat
// 
// If you need to use embeddings in the frontend, call the backend API instead:
// POST /api/chat with { message: "...", context: "..." }

export async function getEmbedding(_text: string): Promise<number[]> {
  throw new Error("Embeddings are handled by FastAPI backend. Use /api/chat endpoint instead.");
}

export async function storeChunk(
  _content: string,
  _module?: string,
  _section?: string
): Promise<string> {
  throw new Error("Chunk storage is handled by FastAPI backend.");
}

export async function searchRelevantChunks(
  _query: string,
  _limit: number = 5
): Promise<Array<{ text: string; score: number; module?: string }>> {
  throw new Error("Vector search is handled by FastAPI backend. Use /api/chat endpoint instead.");
}

