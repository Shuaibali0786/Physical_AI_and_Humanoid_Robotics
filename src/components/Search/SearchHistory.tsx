/**
 * Search History Component
 *
 * This component displays the user's search history and allows for quick access
 * to previous searches.
 */

import React, { useState, useEffect } from 'react';
import { SearchHistoryItem } from '../../utils/storage';
import { useSearch } from '../../hooks/useSearch';
import './../../styles/translation-search.css';

interface SearchHistoryProps {
  onSearchSelect: (query: string) => void;
  limit?: number;
  showClearButton?: boolean;
}

const SearchHistory: React.FC<SearchHistoryProps> = ({
  onSearchSelect,
  limit = 5,
  showClearButton = true
}) => {
  const { getSearchHistory, clearSearchHistory } = useSearch();
  const [history, setHistory] = useState<SearchHistoryItem[]>([]);
  const [isHistoryVisible, setIsHistoryVisible] = useState<boolean>(true);

  // Load search history
  useEffect(() => {
    const loadedHistory = getSearchHistory(limit);
    setHistory(loadedHistory);
  }, [getSearchHistory, limit]);

  // Refresh history when needed
  const refreshHistory = () => {
    const updatedHistory = getSearchHistory(limit);
    setHistory(updatedHistory);
  };

  // Handle clear history
  const handleClearHistory = () => {
    const success = clearSearchHistory();
    if (success) {
      setHistory([]);
    }
  };

  // Format date for display
  const formatDate = (timestamp: number): string => {
    const date = new Date(timestamp);
    return date.toLocaleString([], {
      month: 'short',
      day: 'numeric',
      hour: '2-digit',
      minute: '2-digit'
    });
  };

  return (
    <div className="search-history-container">
      <div className="search-history-header">
        <h3 className="search-history-title">Recent Searches</h3>
        {showClearButton && history.length > 0 && (
          <button
            className="clear-history-button"
            onClick={handleClearHistory}
            title="Clear search history"
          >
            Clear
          </button>
        )}
      </div>

      {history.length > 0 ? (
        <ul className="search-history-list">
          {history.map((item: SearchHistoryItem) => (
            <li
              key={item.id}
              className="search-history-item"
              onClick={() => onSearchSelect(item.query)}
            >
              <div className="search-history-query">
                {item.query}
              </div>
              <div className="search-history-date">
                {formatDate(item.timestamp)}
              </div>
            </li>
          ))}
        </ul>
      ) : (
        <p className="no-search-history">No search history available</p>
      )}
    </div>
  );
};

export default SearchHistory;