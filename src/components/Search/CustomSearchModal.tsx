/**
 * Custom Search Modal Component
 *
 * This component provides a modal search interface for searching across book modules.
 */

import React, { useState, useEffect, useRef } from 'react';
import { useSearch } from '../../hooks/useSearch';
import { SearchResult } from '../../services/searchService';
import './../../styles/translation-search.css';

interface CustomSearchModalProps {
  isOpen: boolean;
  onClose: () => void;
}

const CustomSearchModal: React.FC<CustomSearchModalProps> = ({
  isOpen,
  onClose
}) => {
  const {
    isSearching,
    results,
    error,
    search,
    getSearchHistory,
    currentQuery,
    searchTime
  } = useSearch();

  const [query, setQuery] = useState<string>('');
  const [showHistory, setShowHistory] = useState<boolean>(false);
  const inputRef = useRef<HTMLInputElement>(null);

  // Focus the input when modal opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  // Handle search submission
  const handleSearch = async (e: React.FormEvent) => {
    e.preventDefault();
    if (query.trim()) {
      await search(query);
      setShowHistory(false);
    }
  };

  // Handle search history item click
  const handleHistoryItemClick = async (historyQuery: string) => {
    setQuery(historyQuery);
    await search(historyQuery);
    setShowHistory(false);
  };

  // Handle keydown events for keyboard navigation
  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Escape') {
      onClose();
    }
  };

  // Close modal when clicking on backdrop
  const handleBackdropClick = (e: React.MouseEvent) => {
    if (e.target === e.currentTarget) {
      onClose();
    }
  };

  if (!isOpen) {
    return null;
  }

  return (
    <div className="search-modal" onClick={handleBackdropClick} onKeyDown={handleKeyDown}>
      <div className="search-modal-content">
        <div className="search-modal-header">
          <h2>Search Book Modules</h2>
          <button className="search-modal-close" onClick={onClose}>
            &times;
          </button>
        </div>

        <div className="search-modal-body">
          <form onSubmit={handleSearch} className="search-form">
            <div className="search-input-container">
              <input
                ref={inputRef}
                type="text"
                value={query}
                onChange={(e) => setQuery(e.target.value)}
                placeholder="Search across book modules..."
                className="search-input"
                autoComplete="off"
              />
            </div>
          </form>

          {error && (
            <div className="error-message">
              {error}
            </div>
          )}

          {isSearching && (
            <div className="search-loading">
              <p>Searching...</p>
            </div>
          )}

          {results.length > 0 && !isSearching && (
            <div className="search-results-container">
              <div className="search-results-info">
                <p>
                  Found {results.length} result{results.length !== 1 ? 's' : ''}
                  {searchTime && ` in ${searchTime}ms`}
                </p>
              </div>
              <ul className="search-results">
                {results.map((result: SearchResult, index) => (
                  <li key={result.id || index} className="search-result-item">
                    <a
                      href={result.url}
                      className="search-result-title"
                      onClick={(e) => {
                        // We'll handle navigation in a way that preserves the search context
                        e.preventDefault();
                        window.location.href = result.url;
                        onClose();
                      }}
                    >
                      {result.title}
                    </a>
                    <div className="search-result-module">
                      Module: {result.module}
                    </div>
                    <div className="search-result-content">
                      {result.content.substring(0, 200)}...
                    </div>
                  </li>
                ))}
              </ul>
            </div>
          )}

          {results.length === 0 && !isSearching && query && (
            <div className="search-no-results">
              <div className="search-no-results-icon">🔍</div>
              <h3>No results found</h3>
              <p>No matches found for "{query}". Try different keywords.</p>
              <div className="search-suggestions">
                <h4>Suggestions:</h4>
                <ul>
                  <li>Check your spelling</li>
                  <li>Try more general terms</li>
                  <li>Try different phrasing</li>
                  <li>Search for specific concepts like "digital twin", "gazebo", "unity", "urdf", "robotics"</li>
                </ul>
              </div>
            </div>
          )}

          {!query && !isSearching && (
            <div className="search-suggestions">
              <h3>Search Tips</h3>
              <ul>
                <li>Enter keywords related to robotics, AI, or specific topics</li>
                <li>Use quotes for exact phrase matching</li>
                <li>Try different forms of the same word</li>
              </ul>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

export default CustomSearchModal;