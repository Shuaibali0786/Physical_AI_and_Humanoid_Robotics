/**
 * Translation Display Component
 *
 * This component displays content that can be translated from English to Roman Urdu.
 * It provides controls for toggling translation and shows loading states.
 */

import React, { useState, useEffect, useCallback } from 'react';
import { useTranslation } from '../../hooks/useTranslation';
import './../../styles/translation-search.css';

interface TranslationDisplayProps {
  content: string;
  className?: string;
  preserveFormatting?: boolean;
  showControls?: boolean;
  onTranslationComplete?: (translated: boolean) => void;
  autoTranslate?: boolean;
}

const TranslationDisplay: React.FC<TranslationDisplayProps> = ({
  content,
  className = '',
  preserveFormatting = false,
  showControls = true,
  onTranslationComplete,
  autoTranslate = false
}) => {
  const {
    isTranslating,
    translatedText,
    originalText,
    error,
    translate,
    toggleTranslation,
    isTranslated,
    revertToOriginal
  } = useTranslation();

  const [displayedContent, setDisplayedContent] = useState<string>(content);
  const [hasBeenTranslated, setHasBeenTranslated] = useState<boolean>(false);

  // Update original text when content prop changes
  useEffect(() => {
    setDisplayedContent(content);
  }, [content]);

  // Handle translation
  const handleTranslate = async () => {
    if (!isTranslated) {
      const result = await translate(content);
      if (result.success) {
        setHasBeenTranslated(true);
        if (onTranslationComplete) {
          onTranslationComplete(true);
        }
      }
    } else {
      revertToOriginal();
      if (onTranslationComplete) {
        onTranslationComplete(false);
      }
    }
  };

  // Toggle between English and Roman Urdu
  const handleToggle = useCallback(async () => {
    const result = await toggleTranslation(content);
    if (result.success) {
      setHasBeenTranslated(!hasBeenTranslated);
      if (onTranslationComplete) {
        onTranslationComplete(!hasBeenTranslated);
      }
    }
  }, [toggleTranslation, content, hasBeenTranslated, onTranslationComplete]);

  // Update displayed content when translation state changes
  useEffect(() => {
    if (isTranslated && translatedText) {
      setDisplayedContent(translatedText);
    } else {
      setDisplayedContent(content);
    }
  }, [isTranslated, translatedText, content]);

  // Auto-translate if enabled
  useEffect(() => {
    if (autoTranslate && content && !hasBeenTranslated) {
      handleTranslate();
    }
  }, [autoTranslate, content, hasBeenTranslated]);

  // Determine the class for the container
  const containerClasses = [
    'translation-display',
    className,
    isTranslating ? 'translating' : '',
    isTranslated ? 'translated' : ''
  ].filter(Boolean).join(' ');

  return (
    <div className={containerClasses}>
      {error && (
        <div className="error-message">
          Translation Error: {error}
        </div>
      )}

      <div className="translation-content">
        {displayedContent}
      </div>

      {showControls && (
        <div className="translation-controls">
          <button
            onClick={handleToggle}
            disabled={isTranslating}
            className={`translate-toggle-button ${isTranslating ? 'loading' : ''}`}
          >
            {isTranslating ? (
              <>
                <span className="loading-spinner"></span> Translating...
              </>
            ) : isTranslated ? (
              'Switch to English'
            ) : (
              'Switch to Roman Urdu'
            )}
          </button>
        </div>
      )}
    </div>
  );
};

export default TranslationDisplay;