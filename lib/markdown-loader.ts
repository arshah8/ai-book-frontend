import fs from 'fs';
import path from 'path';

/**
 * Load markdown content from Docusaurus docs folder
 * Works in Next.js App Router (server components)
 */
export async function loadMarkdownContent(modulePath: string): Promise<string> {
  try {
    // Path to Docusaurus docs folder
    const docsPath = path.join(process.cwd(), 'book', 'docs', modulePath);
    
    // Check if file exists
    if (!fs.existsSync(docsPath)) {
      console.warn(`Markdown file not found: ${docsPath}`);
      return '';
    }
    
    // Read and return content
    const content = fs.readFileSync(docsPath, 'utf-8');
    return content;
  } catch (error) {
    console.error(`Error loading markdown: ${error}`);
    return '';
  }
}

/**
 * Get all markdown files for a module
 * Works in Next.js App Router (server components)
 */
export async function getModuleFiles(moduleName: string): Promise<string[]> {
  try {
    const modulePath = path.join(process.cwd(), 'book', 'docs', moduleName);
    
    if (!fs.existsSync(modulePath)) {
      return [];
    }
    
    const files = fs.readdirSync(modulePath)
      .filter(file => file.endsWith('.md'))
      .map(file => file.replace('.md', ''));
    
    return files;
  } catch (error) {
    console.error(`Error getting module files: ${error}`);
    return [];
  }
}

