import { QdrantClient } from "@qdrant/qdrant-js";

const qdrantUrl = process.env.QDRANT_URL || "https://your-cluster.qdrant.io";
const qdrantApiKey = process.env.QDRANT_API_KEY || "";

export const qdrantClient = new QdrantClient({
  url: qdrantUrl,
  apiKey: qdrantApiKey,
});

export const COLLECTION_NAME = "book_content";

// Initialize collection if it doesn't exist
export async function ensureCollection() {
  try {
    const collections = await qdrantClient.getCollections();
    const collectionExists = collections.collections.some(
      (c) => c.name === COLLECTION_NAME
    );

    if (!collectionExists) {
      await qdrantClient.createCollection(COLLECTION_NAME, {
        vectors: {
          size: 1536, // OpenAI text-embedding-3-small dimension
          distance: "Cosine",
        },
      });
      console.log(`Created Qdrant collection: ${COLLECTION_NAME}`);
    }
  } catch (error) {
    console.error("Error ensuring Qdrant collection:", error);
    throw error;
  }
}

// Search for similar vectors
export async function searchVectors(
  queryVector: number[],
  limit: number = 5
) {
  try {
    const results = await qdrantClient.search(COLLECTION_NAME, {
      vector: queryVector,
      limit,
      with_payload: true,
    });

    return results.map((result) => ({
      text: result.payload?.text as string,
      score: result.score,
      id: result.id,
    }));
  } catch (error) {
    console.error("Error searching Qdrant:", error);
    throw error;
  }
}

// Add vector to collection
export async function addVector(
  id: string,
  vector: number[],
  payload: { text: string; module?: string; section?: string }
) {
  try {
    await qdrantClient.upsert(COLLECTION_NAME, {
      wait: true,
      points: [
        {
          id,
          vector,
          payload,
        },
      ],
    });
  } catch (error) {
    console.error("Error adding vector to Qdrant:", error);
    throw error;
  }
}

