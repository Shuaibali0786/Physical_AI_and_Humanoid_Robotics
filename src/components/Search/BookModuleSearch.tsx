/**
 * Book Module Search Component
 *
 * This component connects search functionality to book modules,
 * allowing users to search across all book modules.
 */

import React, { useEffect, useState } from 'react';
import { searchService, SearchResult } from '../../services/searchService';
import { useSearch } from '../../hooks/useSearch';
import SearchServiceComponent from './SearchServiceComponent';
import SearchResultDisplay from './SearchResultDisplay';
import './../../styles/translation-search.css';

interface BookModule {
  id: string;
  title: string;
  content: string;
  module: string;
  url: string;
  tags?: string[];
}

interface BookModuleSearchProps {
  modules: BookModule[];
  onSearchComplete?: (results: SearchResult[]) => void;
}

const BookModuleSearch: React.FC<BookModuleSearchProps> = ({
  modules = [],
  onSearchComplete
}) => {
  const {
    isSearching,
    results,
    error,
    search,
    currentQuery
  } = useSearch();

  const [isIndexing, setIsIndexing] = useState<boolean>(true);
  const [indexComplete, setIndexComplete] = useState<boolean>(false);

  // Index all book modules when component mounts
  useEffect(() => {
    const indexModules = async () => {
      setIsIndexing(true);

      try {
        // Index each module
        for (const module of modules) {
          await searchService.indexModule({
            title: module.title,
            content: module.content,
            module: module.module,
            url: module.url,
            tags: module.tags
          });
        }

        setIndexComplete(true);
        console.log(`Indexed ${modules.length} book modules`);
      } catch (err) {
        console.error('Error indexing book modules:', err);
      } finally {
        setIsIndexing(false);
      }
    };

    if (modules.length > 0) {
      indexModules();
    }
  }, [modules]);

  // Handle search completion
  useEffect(() => {
    if (results.length > 0 && onSearchComplete) {
      onSearchComplete(results);
    }
  }, [results, onSearchComplete]);

  return (
    <div className="book-module-search">
      <div className="search-header">
        <h2>Search Book Modules</h2>
        {isIndexing && (
          <div className="indexing-status">
            <span className="loading-spinner"></span> Indexing book modules...
          </div>
        )}
        {indexComplete && !isIndexing && (
          <div className="index-status">
            ✅ Book modules indexed and ready for search
          </div>
        )}
      </div>

      <div className="search-input-section">
        <SearchServiceComponent
          onSearchComplete={onSearchComplete}
          placeholder="Search across all book modules..."
        />
      </div>

      {error && (
        <div className="error-message">
          {error}
        </div>
      )}

      <div className="search-results-section">
        {currentQuery && !isSearching && (
          <h3>
            Search Results for: <span className="search-query">"{currentQuery}"</span>
          </h3>
        )}

        {results.length > 0 && (
          <SearchResultDisplay
            results={results}
            showModuleInfo={true}
            showScore={true}
            onResultClick={(result) => {
              // Navigate to the result's URL
              window.location.href = result.url;
            }}
          />
        )}

        {currentQuery && !isSearching && results.length === 0 && (
          <div className="no-results-message">
            <h3>No results found</h3>
            <p>Could not find any matches for "{currentQuery}" in the book modules.</p>
            <ul>
              <li>Check your spelling</li>
              <li>Try different keywords</li>
              <li>Try more general terms</li>
            </ul>
          </div>
        )}
      </div>

      <div className="search-tips">
        <h3>Search Tips</h3>
        <ul>
          <li>Use specific keywords related to robotics, AI, or the topics you're studying</li>
          <li>Try searching for concepts like "digital twin", "gazebo", "unity", "urdf", etc.</li>
          <li>Use quotes for exact phrase matching</li>
          <li>Results are ranked by relevance to your query</li>
        </ul>
      </div>
    </div>
  );
};

// Function to get all book modules from the filesystem (this would be implemented based on the Docusaurus structure)
export const getAllBookModules = async (): Promise<BookModule[]> => {
  // In a real implementation, this would scan the docs directory and extract module information
  // For now, we'll return an empty array and provide a way to set modules externally
  return [];
};

export default BookModuleSearch;