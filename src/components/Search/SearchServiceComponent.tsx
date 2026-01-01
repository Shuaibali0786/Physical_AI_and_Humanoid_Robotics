/**
 * Search Service Component
 *
 * This component provides enhanced search functionality for book modules,
 * integrating with the search service to provide custom search features.
 */

import React, { useState, useEffect } from 'react';
import { searchService, SearchQuery, SearchResult } from '../../services/searchService';
import { useSearch } from '../../hooks/useSearch';
import './../../styles/translation-search.css';

interface SearchServiceComponentProps {
  onSearchComplete?: (results: SearchResult[]) => void;
  placeholder?: string;
  autoFocus?: boolean;
}

const SearchServiceComponent: React.FC<SearchServiceComponentProps> = ({
  onSearchComplete,
  placeholder = 'Search book modules...',
  autoFocus = false
}) => {
  const {
    isSearching,
    results,
    error,
    search,
    currentQuery
  } = useSearch();

  const [query, setQuery] = useState<string>('');
  const [searchParams, setSearchParams] = useState({
    limit: 10,
    module: '',
    tags: [] as string[]
  });

  // Handle search submission
  const handleSearch = async () => {
    if (query.trim()) {
      const searchQuery: SearchQuery = {
        query,
        limit: searchParams.limit,
        filters: searchParams.module ? { module: searchParams.module } : undefined
      };

      const response = await search(query, searchQuery);
      if (onSearchComplete && response.success) {
        onSearchComplete(response.results);
      }
    }
  };

  // Handle key press (Enter to search)
  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter') {
      handleSearch();
    }
  };

  // Index book module content when component mounts
  useEffect(() => {
    // In a real implementation, we would index the book modules here
    // For now, we'll just log that indexing would happen
    console.log('SearchServiceComponent mounted - would index book modules here');

    // Example of how to index content:
    // const modules = getBookModules(); // This would get all book modules
    // modules.forEach(module => {
    //   searchService.indexModule({
    //     title: module.title,
    //     content: module.content,
    //     module: module.moduleName,
    //     url: module.url,
    //     tags: module.tags
    //   });
    // });
  }, []);

  return (
    <div className="search-service-component">
      <div className="search-input-area">
        <div className="search-input-container">
          <input
            type="text"
            value={query}
            onChange={(e) => setQuery(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder={placeholder}
            className="search-input"
            autoFocus={autoFocus}
          />
          <button
            onClick={handleSearch}
            disabled={isSearching || !query.trim()}
            className="search-button"
          >
            {isSearching ? 'Searching...' : 'Search'}
          </button>
        </div>
      </div>

      {error && (
        <div className="error-message">
          {error}
        </div>
      )}

      {currentQuery && !isSearching && results.length === 0 && (
        <div className="no-results-message">
          No results found for "{currentQuery}"
        </div>
      )}

      {results.length > 0 && (
        <div className="search-results-summary">
          <h3>Search Results</h3>
          <p>Found {results.length} result{results.length !== 1 ? 's' : ''}</p>

          <ul className="search-results-list">
            {results.map((result, index) => (
              <li key={result.id || index} className="search-result-item">
                <a href={result.url} className="search-result-link">
                  <h4 className="search-result-title">{result.title}</h4>
                  <div className="search-result-module">Module: {result.module}</div>
                  <p className="search-result-excerpt">
                    {result.content.substring(0, 150)}...
                  </p>
                </a>
              </li>
            ))}
          </ul>
        </div>
      )}

      <div className="search-options">
        <div className="search-limit-control">
          <label htmlFor="search-limit">Results per page:</label>
          <select
            id="search-limit"
            value={searchParams.limit}
            onChange={(e) => setSearchParams({...searchParams, limit: parseInt(e.target.value)})}
          >
            <option value={5}>5</option>
            <option value={10}>10</option>
            <option value={20}>20</option>
            <option value={50}>50</option>
          </select>
        </div>
      </div>
    </div>
  );
};

export default SearchServiceComponent;