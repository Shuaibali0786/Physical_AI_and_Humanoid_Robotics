/**
 * Translation History Manager Component
 *
 * This component manages the display and management of translation history.
 */

import React, { useState, useEffect } from 'react';
import { TranslationHistoryItem } from '../../utils/storage';
import { useTranslation } from '../../hooks/useTranslation';
import './../../styles/translation-search.css';

interface TranslationHistoryManagerComponentProps {
  limit?: number;
  showActions?: boolean;
}

const TranslationHistoryManagerComponent: React.FC<TranslationHistoryManagerComponentProps> = ({
  limit = 10,
  showActions = true
}) => {
  const { getTranslationHistory, clearTranslationHistory } = useTranslation();
  const [history, setHistory] = useState<TranslationHistoryItem[]>([]);
  const [isLoading, setIsLoading] = useState<boolean>(true);

  // Load history when component mounts
  useEffect(() => {
    loadHistory();
  }, []);

  const loadHistory = async () => {
    setIsLoading(true);
    try {
      const loadedHistory = getTranslationHistory(limit);
      setHistory(loadedHistory);
    } catch (error) {
      console.error('Error loading translation history:', error);
    } finally {
      setIsLoading(false);
    }
  };

  const handleClearHistory = async () => {
    const success = clearTranslationHistory();
    if (success) {
      setHistory([]);
    }
  };

  const formatDate = (timestamp: number): string => {
    const date = new Date(timestamp);
    return date.toLocaleString();
  };

  return (
    <div className="translation-history-manager">
      <div className="translation-history-header">
        <h3>Translation History</h3>
        {showActions && (
          <button
            onClick={handleClearHistory}
            className="clear-history-button"
            disabled={isLoading}
          >
            Clear History
          </button>
        )}
      </div>

      {isLoading ? (
        <div className="loading-history">
          <p>Loading translation history...</p>
        </div>
      ) : history.length === 0 ? (
        <div className="no-history">
          <p>No translation history available.</p>
        </div>
      ) : (
        <div className="history-list">
          <ul className="translation-history-items">
            {history.map((item) => (
              <li key={item.id} className="translation-history-item">
                <div className="history-item-header">
                  <div className="history-item-date">
                    {formatDate(item.timestamp)}
                  </div>
                </div>
                <div className="history-item-content">
                  <div className="history-original">
                    <strong>Original:</strong> {item.originalText.substring(0, 100)}{item.originalText.length > 100 ? '...' : ''}
                  </div>
                  <div className="history-translated">
                    <strong>Translated:</strong> {item.translatedText.substring(0, 100)}{item.translatedText.length > 100 ? '...' : ''}
                  </div>
                </div>
              </li>
            ))}
          </ul>
        </div>
      )}
    </div>
  );
};

export default TranslationHistoryManagerComponent;