/**
 * Search Result Display Component
 *
 * This component displays search results with module context,
 * showing relevant information about where the results were found.
 */

import React from 'react';
import { SearchResult } from '../../services/searchService';
import './../../styles/translation-search.css';

interface SearchResultDisplayProps {
  results: SearchResult[];
  onResultClick?: (result: SearchResult) => void;
  showModuleInfo?: boolean;
  showScore?: boolean;
  maxContentLength?: number;
  navigateToResult?: (url: string, title: string) => void;
}

const SearchResultDisplay: React.FC<SearchResultDisplayProps> = ({
  results,
  onResultClick,
  showModuleInfo = true,
  showScore = false,
  maxContentLength = 200,
  navigateToResult
}) => {
  const handleResultClick = (result: SearchResult) => {
    if (onResultClick) {
      onResultClick(result);
    }

    // Navigate to the result if navigation function is provided
    if (navigateToResult) {
      navigateToResult(result.url, result.title);
    } else {
      // Default navigation behavior
      window.location.href = result.url;
    }
  };

  // Truncate content to specified length
  const truncateContent = (content: string, maxLength: number): string => {
    if (content.length <= maxLength) {
      return content;
    }
    return content.substring(0, maxLength) + '...';
  };

  return (
    <div className="search-result-display">
      {results.length === 0 ? (
        <div className="no-results">
          <p>No search results found.</p>
        </div>
      ) : (
        <ul className="search-results-list">
          {results.map((result, index) => (
            <li
              key={result.id || index}
              className="search-result-item"
              onClick={() => handleResultClick(result)}
            >
              <div className="search-result-content">
                <a
                  href={result.url}
                  className="search-result-title"
                  onClick={(e) => {
                    e.preventDefault();
                    handleResultClick(result);
                  }}
                >
                  {result.title}
                </a>

                {showModuleInfo && result.module && (
                  <div className="search-result-module">
                    <span className="module-label">Module:</span> {result.module}
                  </div>
                )}

                {result.content && (
                  <div className="search-result-excerpt">
                    {truncateContent(result.content, maxContentLength)}
                  </div>
                )}

                {showScore && result.score !== undefined && (
                  <div className="search-result-score">
                    <span className="score-label">Relevance:</span> {Math.round(result.score * 100) / 100}
                  </div>
                )}

                {result.tags && result.tags.length > 0 && (
                  <div className="search-result-tags">
                    {result.tags.map((tag, tagIndex) => (
                      <span key={tagIndex} className="search-tag">
                        {tag}
                      </span>
                    ))}
                  </div>
                )}
              </div>
            </li>
          ))}
        </ul>
      )}
    </div>
  );
};

export default SearchResultDisplay;