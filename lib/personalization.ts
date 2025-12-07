import { db, userProfiles } from "./database";
import { eq } from "drizzle-orm";

export type ExperienceLevel = "beginner" | "intermediate" | "advanced";

export interface PersonalizationConfig {
  showAdvancedTopics: boolean;
  showCodeExamples: boolean;
  codeComplexity: "simple" | "standard" | "advanced";
  explanationDepth: "basic" | "detailed" | "comprehensive";
}

export async function getPersonalizationConfig(
  userId: string
): Promise<PersonalizationConfig> {
  try {
    const profile = await db
      .select()
      .from(userProfiles)
      .where(eq(userProfiles.userId, userId))
      .limit(1);

    if (profile.length === 0) {
      return getDefaultConfig();
    }

    const userProfile = profile[0];
    const experienceLevel = userProfile.experienceLevel || "beginner";

    return getConfigForLevel(experienceLevel as ExperienceLevel);
  } catch (error) {
    console.error("Error getting personalization config:", error);
    return getDefaultConfig();
  }
}

function getConfigForLevel(level: ExperienceLevel): PersonalizationConfig {
  switch (level) {
    case "beginner":
      return {
        showAdvancedTopics: false,
        showCodeExamples: true,
        codeComplexity: "simple",
        explanationDepth: "detailed",
      };
    case "intermediate":
      return {
        showAdvancedTopics: true,
        showCodeExamples: true,
        codeComplexity: "standard",
        explanationDepth: "detailed",
      };
    case "advanced":
      return {
        showAdvancedTopics: true,
        showCodeExamples: true,
        codeComplexity: "advanced",
        explanationDepth: "comprehensive",
      };
    default:
      return getDefaultConfig();
  }
}

function getDefaultConfig(): PersonalizationConfig {
  return {
    showAdvancedTopics: true,
    showCodeExamples: true,
    codeComplexity: "standard",
    explanationDepth: "detailed",
  };
}

// Get personalized content based on user profile
export function personalizeContent(
  content: string,
  config: PersonalizationConfig
): string {
  // This is a simple implementation
  // In a full implementation, you'd have different content variants
  // and select based on the config
  
  if (!config.showAdvancedTopics) {
    // Remove advanced sections marked with <!-- ADVANCED -->
    content = content.replace(/<!-- ADVANCED -->[\s\S]*?<!-- \/ADVANCED -->/g, "");
  }

  return content;
}

