/**
 * Search Hook
 *
 * This React hook provides search functionality to components,
 * including state management for search results and history.
 */

import { useState, useEffect, useCallback, useMemo } from 'react';
import { searchService, SearchQuery, SearchResponse, SearchResult } from '../services/searchService';
import { SearchHistoryItem } from '../utils/storage';

export interface UseSearchOptions {
  defaultLimit?: number;
  enableCache?: boolean;
  enableHistory?: boolean;
}

export interface UseSearchReturn {
  isSearching: boolean;
  results: SearchResult[];
  totalResults: number;
  error: string | null;
  search: (query: string, options?: Partial<SearchQuery>) => Promise<SearchResponse>;
  getSearchHistory: (limit?: number) => SearchHistoryItem[];
  clearSearchHistory: () => boolean;
  addToSearchHistory: (query: string, resultsCount: number) => void;
  currentQuery: string;
  searchTime: number | null;
  loadMore: (additionalResults?: number) => void;
  hasMoreResults: boolean;
}

export const useSearch = (options: UseSearchOptions = {}): UseSearchReturn => {
  const {
    defaultLimit = 10,
    enableCache = true,
    enableHistory = true
  } = options;

  const [isSearching, setIsSearching] = useState<boolean>(false);
  const [results, setResults] = useState<SearchResult[]>([]);
  const [totalResults, setTotalResults] = useState<number>(0);
  const [error, setError] = useState<string | null>(null);
  const [currentQuery, setCurrentQuery] = useState<string>('');
  const [searchTime, setSearchTime] = useState<number | null>(null);
  const [offset, setOffset] = useState<number>(0);
  const [limit, setLimit] = useState<number>(defaultLimit);

  // Calculate if there are more results available
  const hasMoreResults = useMemo(() => {
    return results.length < totalResults;
  }, [results.length, totalResults]);

  // Search function
  const search = useCallback(async (query: string, searchOptions?: Partial<SearchQuery>): Promise<SearchResponse> => {
    setIsSearching(true);
    setError(null);
    setCurrentQuery(query);

    try {
      const searchQuery: SearchQuery = {
        query,
        limit: searchOptions?.limit || limit,
        offset: searchOptions?.offset || offset,
        filters: searchOptions?.filters,
        ...searchOptions
      };

      const response = await searchService.search(searchQuery);

      if (response.success) {
        setResults(response.results);
        setTotalResults(response.totalResults);
        setSearchTime(response.took);
      } else {
        setError(response.error || 'Search failed');
      }

      return response;
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'An unknown error occurred';
      setError(errorMessage);
      console.error('Search error:', err);

      // Return a response object even in case of error
      return {
        query,
        results: [],
        totalResults: 0,
        took: 0,
        success: false,
        error: errorMessage
      };
    } finally {
      setIsSearching(false);
    }
  }, [limit, offset]);

  // Load more results
  const loadMore = useCallback((additionalResults: number = defaultLimit) => {
    const newOffset = offset + limit;
    setOffset(newOffset);
    setLimit(additionalResults);

    // Perform a new search with the updated offset
    if (currentQuery) {
      search(currentQuery, {
        limit: additionalResults,
        offset: newOffset
      });
    }
  }, [offset, limit, defaultLimit, currentQuery, search]);

  // Get search history
  const getSearchHistory = useCallback((limit?: number): SearchHistoryItem[] => {
    if (!enableHistory) {
      return [];
    }
    return searchService.getSearchHistory(limit);
  }, [enableHistory]);

  // Clear search history
  const clearSearchHistory = useCallback((): boolean => {
    if (!enableHistory) {
      return false;
    }
    return searchService.clearSearchHistory();
  }, [enableHistory]);

  // Add to search history (manually)
  const addToSearchHistory = useCallback((query: string, resultsCount: number) => {
    if (!enableHistory) {
      return;
    }
    // This is handled automatically by the search service, but we expose it for manual use
  }, [enableHistory]);

  // Reset pagination when a new search is performed
  useEffect(() => {
    if (currentQuery) {
      setOffset(0);
    }
  }, [currentQuery]);

  // Reset results when query changes significantly
  useEffect(() => {
    if (!currentQuery) {
      setResults([]);
      setTotalResults(0);
      setSearchTime(null);
    }
  }, [currentQuery]);

  return {
    isSearching,
    results,
    totalResults,
    error,
    search,
    getSearchHistory,
    clearSearchHistory,
    addToSearchHistory,
    currentQuery,
    searchTime,
    loadMore,
    hasMoreResults
  };
};

export default useSearch;