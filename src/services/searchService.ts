/**
 * Search Service
 *
 * This service provides the core functionality for searching across book modules,
 * including indexing, search history management, and result processing.
 */

import { searchHistoryStorage, SearchHistoryItem } from '../utils/storage';

export interface SearchQuery {
  query: string;
  limit?: number;
  offset?: number;
  filters?: {
    module?: string;
    tags?: string[];
    dateRange?: { start: Date; end: Date };
  };
}

export interface SearchResult {
  id: string;
  title: string;
  content: string;
  module: string;
  url: string;
  score: number;
  tags?: string[];
  timestamp?: number;
}

export interface SearchResponse {
  query: string;
  results: SearchResult[];
  totalResults: number;
  took: number; // Time taken in milliseconds
  success: boolean;
  error?: string;
}

export interface SearchServiceInterface {
  search(query: SearchQuery): Promise<SearchResponse>;
  indexContent(content: { id: string; title: string; content: string; module: string; url: string; tags?: string[] }): Promise<boolean>;
  indexModule(moduleData: { title: string; content: string; module: string; url: string; tags?: string[] }): Promise<boolean>;
  getSearchHistory(limit?: number): SearchHistoryItem[];
  clearSearchHistory(): boolean;
}

export class SearchService implements SearchServiceInterface {
  private searchCache: Map<string, { results: SearchResult[]; timestamp: number }> = new Map();
  private readonly CACHE_TTL = 5 * 60 * 1000; // 5 minutes cache TTL
  private readonly API_BASE_URL = typeof window !== 'undefined'
    ? (window as any).env?.NEXT_PUBLIC_SEARCH_API_URL || 'http://localhost:8001'
    : (process.env.NEXT_PUBLIC_SEARCH_API_URL || 'http://localhost:8001');

  async search(query: SearchQuery): Promise<SearchResponse> {
    const startTime = Date.now();

    try {
      // Validate input
      if (!query.query || typeof query.query !== 'string' || query.query.trim().length === 0) {
        return {
          query: query.query || '',
          results: [],
          totalResults: 0,
          took: Date.now() - startTime,
          success: false,
          error: 'Invalid search query provided'
        };
      }

      // Check cache first
      const cacheKey = this.getCacheKey(query);
      try {
        const cached = this.getCachedResults(cacheKey);
        if (cached) {
          // Add to search history
          this.addToSearchHistory(query.query, cached.results.length);

          return {
            query: query.query,
            results: cached.results,
            totalResults: cached.results.length,
            took: Date.now() - startTime,
            success: true
          };
        }
      } catch (cacheError) {
        console.error('Error accessing search cache:', cacheError);
        // Continue with search even if cache fails
      }

      // Perform the search via API call
      let results: SearchResult[] = [];
      try {
        // Call the backend search API
        const response = await fetch(`${this.API_BASE_URL}/api/search`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            query: query.query,
            search_type: 'hybrid', // Use hybrid search by default
            top_k: query.limit || 10
          })
        });

        if (!response.ok) {
          throw new Error(`Search API error: ${response.status} ${response.statusText}`);
        }

        const apiResponse = await response.json();

        if (!apiResponse.success) {
          throw new Error(apiResponse.error || 'Search API returned error');
        }

        // Transform API results to our SearchResult format
        results = apiResponse.results.map((item: any, index: number) => ({
          id: item.id || `result-${index}`,
          title: item.title || 'Untitled',
          content: item.content || item.payload?.content || '',
          module: item.module || 'General',
          url: item.url || item.payload?.url || '#',
          score: item.score || 0,
          tags: item.tags || [],
          timestamp: item.timestamp || Date.now()
        }));
      } catch (searchError) {
        console.error('Error performing search:', searchError);
        return {
          query: query.query || '',
          results: [],
          totalResults: 0,
          took: Date.now() - startTime,
          success: false,
          error: `Search process failed: ${searchError instanceof Error ? searchError.message : 'Unknown error'}`
        };
      }

      // Cache the results
      try {
        this.setCachedResults(cacheKey, results);
      } catch (cacheError) {
        console.error('Error caching search results:', cacheError);
        // Don't fail the search if caching fails
      }

      // Add to search history
      try {
        this.addToSearchHistory(query.query, results.length);
      } catch (historyError) {
        console.error('Error adding search to history:', historyError);
        // Don't fail the search if history storage fails
      }

      return {
        query: query.query,
        results,
        totalResults: results.length,
        took: Date.now() - startTime,
        success: true
      };
    } catch (error) {
      console.error('Search service error:', error);
      return {
        query: query.query || '',
        results: [],
        totalResults: 0,
        took: Date.now() - startTime,
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error occurred during search'
      };
    }
  }

  async indexContent(content: { id: string; title: string; content: string; module: string; url: string; tags?: string[] }): Promise<boolean> {
    try {
      // In the new architecture, indexing happens in the backend
      // This method is now a no-op but kept for interface compatibility
      console.warn('Indexing is handled by the backend service. This method is deprecated.');
      return true;
    } catch (error) {
      console.error('Error in indexContent (deprecated):', error);
      return false;
    }
  }

  async indexModule(moduleData: { title: string; content: string; module: string; url: string; tags?: string[] }): Promise<boolean> {
    try {
      // In the new architecture, indexing happens in the backend
      // This method is now a no-op but kept for interface compatibility
      console.warn('Indexing is handled by the backend service. This method is deprecated.');
      return true;
    } catch (error) {
      console.error('Error in indexModule (deprecated):', error);
      return false;
    }
  }

  getSearchHistory(limit?: number): SearchHistoryItem[] {
    return searchHistoryStorage.getSearchHistory(limit);
  }

  clearSearchHistory(): boolean {
    return searchHistoryStorage.clearSearchHistory();
  }

  private getCacheKey(query: SearchQuery): string {
    return JSON.stringify({
      query: query.query,
      limit: query.limit,
      offset: query.offset,
      filters: query.filters
    });
  }

  private getCachedResults(cacheKey: string): { results: SearchResult[]; timestamp: number } | null {
    const cached = this.searchCache.get(cacheKey);
    if (cached && Date.now() - cached.timestamp < this.CACHE_TTL) {
      return cached;
    }
    // Remove expired cache
    this.searchCache.delete(cacheKey);
    return null;
  }

  private setCachedResults(cacheKey: string, results: SearchResult[]): void {
    this.searchCache.set(cacheKey, {
      results,
      timestamp: Date.now()
    });
  }

  private addToSearchHistory(query: string, resultsCount: number): void {
    searchHistoryStorage.addSearch(query, resultsCount);
  }

  private generateId(title: string, url: string): string {
    // Generate a unique ID based on title and URL
    const baseString = `${title}-${url}-${Date.now()}`;
    // Simple hash function to create a unique ID
    let hash = 0;
    for (let i = 0; i < baseString.length; i++) {
      const char = baseString.charCodeAt(i);
      hash = ((hash << 5) - hash) + char;
      hash = hash & hash; // Convert to 32-bit integer
    }
    return Math.abs(hash).toString(36) + Date.now().toString(36);
  }

  /**
   * Load index from storage (if available)
   */
  async loadIndex(): Promise<boolean> {
    try {
      // In a real implementation, this would load the index from storage
      // For now, we'll just return true as the index is in-memory
      return true;
    } catch (error) {
      console.error('Error loading index:', error);
      return false;
    }
  }

  /**
   * Save index to storage (if available)
   */
  async saveIndex(): Promise<boolean> {
    try {
      // In a real implementation, this would save the index to storage
      // For now, we'll just return true as the index is in-memory
      return true;
    } catch (error) {
      console.error('Error saving index:', error);
      return false;
    }
  }
}

// Export a singleton instance of the search service
export const searchService = new SearchService();

export default searchService;