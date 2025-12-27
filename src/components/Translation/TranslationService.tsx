/**
 * Translation Service Component
 *
 * This component provides client-side conversion functionality
 * for English to Roman Urdu translation.
 */

import React, { useState } from 'react';
import { englishToRomanUrdu } from '../../utils/transliterate';
import { translationHistoryStorage, TranslationHistoryItem } from '../../utils/storage';
import './../../styles/translation-search.css';

interface TranslationServiceProps {
  onTranslate?: (original: string, translated: string) => void;
  className?: string;
}

const TranslationService: React.FC<TranslationServiceProps> = ({
  onTranslate,
  className = ''
}) => {
  const [inputText, setInputText] = useState<string>('');
  const [translatedText, setTranslatedText] = useState<string>('');
  const [isTranslating, setIsTranslating] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);

  const handleTranslate = async () => {
    if (!inputText.trim()) {
      setError('Please enter text to translate');
      return;
    }

    setIsTranslating(true);
    setError(null);

    try {
      // Perform the translation
      const result = englishToRomanUrdu(inputText);
      setTranslatedText(result);

      // Add to history
      const historyItem: TranslationHistoryItem = {
        id: Date.now().toString() + Math.random().toString(36).substr(2, 9),
        originalText: inputText,
        translatedText: result,
        timestamp: Date.now(),
        sourceLanguage: 'en',
        targetLanguage: 'roman-urdu'
      };

      translationHistoryStorage.addItem(historyItem);

      // Call the callback if provided
      if (onTranslate) {
        onTranslate(inputText, result);
      }
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'An error occurred during translation';
      setError(errorMessage);
      console.error('Translation error:', err);
    } finally {
      setIsTranslating(false);
    }
  };

  const handleClear = () => {
    setInputText('');
    setTranslatedText('');
    setError(null);
  };

  return (
    <div className={`translation-service ${className}`}>
      <div className="translation-input-container">
        <textarea
          value={inputText}
          onChange={(e) => setInputText(e.target.value)}
          placeholder="Enter English text to translate to Roman Urdu..."
          className="translation-input"
          rows={4}
        />
      </div>

      <div className="translation-controls">
        <button
          onClick={handleTranslate}
          disabled={isTranslating || !inputText.trim()}
          className={`translate-button ${isTranslating ? 'loading' : ''}`}
        >
          {isTranslating ? (
            <>
              <span className="loading-spinner"></span> Translating...
            </>
          ) : (
            'Translate'
          )}
        </button>
        <button
          onClick={handleClear}
          className="clear-button"
        >
          Clear
        </button>
      </div>

      {error && (
        <div className="error-message">
          {error}
        </div>
      )}

      {translatedText && (
        <div className="translation-output-container">
          <h3>Translated Text:</h3>
          <div className="translation-output">
            {translatedText}
          </div>
        </div>
      )}
    </div>
  );
};

export default TranslationService;