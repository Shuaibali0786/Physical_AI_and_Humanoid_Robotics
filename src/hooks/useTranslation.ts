/**
 * Translation Hook
 *
 * This React hook provides translation functionality to components,
 * including state management for translation status and history.
 */

import { useState, useEffect, useCallback } from 'react';
import { translationService, TranslationRequest, TranslationResponse } from '../services/translationService';
import { TranslationHistoryItem } from '../utils/storage';

export interface UseTranslationOptions {
  autoTranslate?: boolean;
  preserveFormatting?: boolean;
  sourceLanguage?: string;
  targetLanguage?: string;
}

export interface UseTranslationReturn {
  isTranslating: boolean;
  translatedText: string;
  originalText: string;
  error: string | null;
  translate: (text: string, options?: Partial<TranslationRequest>) => Promise<TranslationResponse>;
  getTranslationHistory: (limit?: number) => TranslationHistoryItem[];
  clearTranslationHistory: () => boolean;
  toggleTranslation: (text: string, options?: Partial<TranslationRequest>) => Promise<TranslationResponse>;
  isTranslated: boolean;
  revertToOriginal: () => void;
}

export const useTranslation = (options: UseTranslationOptions = {}): UseTranslationReturn => {
  const {
    autoTranslate = false,
    preserveFormatting = false,
    sourceLanguage = 'en',
    targetLanguage = 'roman-urdu'
  } = options;

  const [isTranslating, setIsTranslating] = useState<boolean>(false);
  const [translatedText, setTranslatedText] = useState<string>('');
  const [originalText, setOriginalText] = useState<string>('');
  const [error, setError] = useState<string | null>(null);
  const [isTranslated, setIsTranslated] = useState<boolean>(false);

  // Translate function
  const translate = useCallback(async (text: string, requestOptions?: Partial<TranslationRequest>): Promise<TranslationResponse> => {
    setIsTranslating(true);
    setError(null);

    try {
      const request: TranslationRequest = {
        text,
        sourceLanguage: requestOptions?.sourceLanguage || sourceLanguage,
        targetLanguage: requestOptions?.targetLanguage || targetLanguage,
        preserveFormatting: requestOptions?.preserveFormatting ?? preserveFormatting,
        ...requestOptions
      };

      const response = await translationService.translate(request);

      if (response.success) {
        setOriginalText(text);
        setTranslatedText(response.translatedText);
        setIsTranslated(true);
      } else {
        setError(response.error || 'Translation failed');
      }

      return response;
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'An unknown error occurred';
      setError(errorMessage);
      console.error('Translation error:', err);

      // Return a response object even in case of error
      return {
        originalText: text,
        translatedText: '',
        sourceLanguage: requestOptions?.sourceLanguage || sourceLanguage,
        targetLanguage: requestOptions?.targetLanguage || targetLanguage,
        timestamp: Date.now(),
        success: false,
        error: errorMessage
      };
    } finally {
      setIsTranslating(false);
    }
  }, [sourceLanguage, targetLanguage, preserveFormatting]);

  // Toggle translation between original and translated text
  const toggleTranslation = useCallback(async (text: string, requestOptions?: Partial<TranslationRequest>): Promise<TranslationResponse> => {
    if (isTranslated && text === translatedText) {
      // If currently showing translation and the same text is passed, revert to original
      setTranslatedText(originalText);
      setIsTranslated(false);

      // Return a response indicating the toggle
      return {
        originalText: originalText,
        translatedText: originalText,
        sourceLanguage: sourceLanguage,
        targetLanguage: sourceLanguage,
        timestamp: Date.now(),
        success: true
      };
    } else {
      // Otherwise, translate the text
      return await translate(text, requestOptions);
    }
  }, [isTranslated, translatedText, originalText, translate, sourceLanguage]);

  // Revert to original text
  const revertToOriginal = useCallback(() => {
    setTranslatedText(originalText);
    setIsTranslated(false);
  }, [originalText]);

  // Get translation history
  const getTranslationHistory = useCallback((limit?: number): TranslationHistoryItem[] => {
    return translationService.getTranslationHistory(limit);
  }, []);

  // Clear translation history
  const clearTranslationHistory = useCallback((): boolean => {
    return translationService.clearTranslationHistory();
  }, []);

  // Auto-translate effect
  useEffect(() => {
    if (autoTranslate && originalText) {
      const autoTranslateText = async () => {
        await translate(originalText);
      };
      autoTranslateText();
    }
  }, [autoTranslate, originalText, translate]);

  return {
    isTranslating,
    translatedText,
    originalText,
    error,
    translate,
    getTranslationHistory,
    clearTranslationHistory,
    toggleTranslation,
    isTranslated,
    revertToOriginal
  };
};

export default useTranslation;