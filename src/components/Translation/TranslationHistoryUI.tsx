/**
 * Translation History UI Component
 *
 * This component provides a user interface for accessing and managing translation history.
 */

import React, { useState, useEffect } from 'react';
import { TranslationHistoryItem } from '../../utils/storage';
import { useTranslation } from '../../hooks/useTranslation';
import TranslationHistoryManagerComponent from './TranslationHistoryManagerComponent';
import './../../styles/translation-search.css';

interface TranslationHistoryUIProps {
  isOpen: boolean;
  onClose: () => void;
}

const TranslationHistoryUI: React.FC<TranslationHistoryUIProps> = ({
  isOpen,
  onClose
}) => {
  const { getTranslationHistory, clearTranslationHistory } = useTranslation();
  const [history, setHistory] = useState<TranslationHistoryItem[]>([]);
  const [searchQuery, setSearchQuery] = useState<string>('');
  const [filteredHistory, setFilteredHistory] = useState<TranslationHistoryItem[]>([]);

  // Load history when component mounts
  useEffect(() => {
    if (isOpen) {
      loadHistory();
    }
  }, [isOpen]);

  const loadHistory = async () => {
    try {
      const loadedHistory = getTranslationHistory();
      setHistory(loadedHistory);
      setFilteredHistory(loadedHistory);
    } catch (error) {
      console.error('Error loading translation history:', error);
    }
  };

  // Filter history based on search query
  useEffect(() => {
    if (!searchQuery) {
      setFilteredHistory(history);
    } else {
      const query = searchQuery.toLowerCase();
      const filtered = history.filter(item =>
        item.originalText.toLowerCase().includes(query) ||
        item.translatedText.toLowerCase().includes(query)
      );
      setFilteredHistory(filtered);
    }
  }, [searchQuery, history]);

  const handleClearHistory = async () => {
    const success = clearTranslationHistory();
    if (success) {
      setHistory([]);
      setFilteredHistory([]);
    }
  };

  const handleExportHistory = () => {
    // In a real implementation, this would export the history to a file
    const historyJson = JSON.stringify(history, null, 2);
    const blob = new Blob([historyJson], { type: 'application/json' });
    const url = URL.createObjectURL(blob);

    const a = document.createElement('a');
    a.href = url;
    a.download = 'translation-history.json';
    document.body.appendChild(a);
    a.click();

    // Clean up
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  };

  if (!isOpen) {
    return null;
  }

  return (
    <div className="translation-history-modal">
      <div className="translation-history-content">
        <div className="translation-history-header">
          <h2>Translation History</h2>
          <button className="history-close-button" onClick={onClose}>
            &times;
          </button>
        </div>

        <div className="translation-history-body">
          <div className="history-search">
            <input
              type="text"
              value={searchQuery}
              onChange={(e) => setSearchQuery(e.target.value)}
              placeholder="Search in translation history..."
              className="history-search-input"
            />
          </div>

          <div className="history-actions">
            <button onClick={handleClearHistory} className="clear-history-button">
              Clear All
            </button>
            <button onClick={handleExportHistory} className="export-history-button">
              Export
            </button>
          </div>

          <div className="history-items-container">
            {filteredHistory.length === 0 ? (
              <div className="no-history-items">
                {searchQuery ? (
                  <p>No matching translations found for "{searchQuery}"</p>
                ) : (
                  <p>No translation history available</p>
                )}
              </div>
            ) : (
              <div className="history-items">
                {filteredHistory.map((item, index) => (
                  <div key={item.id} className="history-item">
                    <div className="history-item-header">
                      <div className="history-item-date">
                        {new Date(item.timestamp).toLocaleString()}
                      </div>
                    </div>
                    <div className="history-item-content">
                      <div className="history-original">
                        <strong>Original:</strong>
                        <p>{item.originalText}</p>
                      </div>
                      <div className="history-translated">
                        <strong>Translated:</strong>
                        <p>{item.translatedText}</p>
                      </div>
                    </div>
                  </div>
                ))}
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
};

export default TranslationHistoryUI;