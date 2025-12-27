/**
 * Translation Service
 *
 * This service provides the core functionality for translating content
 * from English to Roman Urdu, including history management and error handling.
 */

import { transliterateToRomanUrdu } from '../utils/transliterate';
import { translationHistoryStorage, TranslationHistoryItem } from '../utils/storage';

export interface TranslationRequest {
  text: string;
  sourceLanguage: string;
  targetLanguage: string;
  preserveFormatting?: boolean;
  maxChunkSize?: number; // Maximum size of text chunks to process at once
}

export interface TranslationResponse {
  originalText: string;
  translatedText: string;
  sourceLanguage: string;
  targetLanguage: string;
  timestamp: number;
  success: boolean;
  error?: string;
}

export interface TranslationServiceInterface {
  translate(request: TranslationRequest): Promise<TranslationResponse>;
  translateBatch(requests: TranslationRequest[]): Promise<TranslationResponse[]>;
  getTranslationHistory(limit?: number): TranslationHistoryItem[];
  clearTranslationHistory(): boolean;
}

export class TranslationService implements TranslationServiceInterface {
  private readonly SUPPORTED_SOURCE_LANGUAGES = ['en', 'english'];
  private readonly SUPPORTED_TARGET_LANGUAGES = ['roman-urdu', 'romanurdu', 'urdu'];
  // private readonly API_BASE_URL = process.env.NEXT_PUBLIC_TRANSLATION_API_URL || 'http://localhost:8001';
private readonly API_BASE_URL = typeof window !== 'undefined'
    ? (window as any).env?.NEXT_PUBLIC_TRANSLATION_API_URL || 'http://localhost:8001'
    : (process.env.NEXT_PUBLIC_TRANSLATION_API_URL || 'http://localhost:8001');

  async translate(request: TranslationRequest): Promise<TranslationResponse> {
    try {
      // Validate input
      if (!request.text || typeof request.text !== 'string') {
        return {
          originalText: '',
          translatedText: '',
          sourceLanguage: request.sourceLanguage,
          targetLanguage: request.targetLanguage,
          timestamp: Date.now(),
          success: false,
          error: 'Invalid input text provided'
        };
      }

      // Validate source language
      if (!this.SUPPORTED_SOURCE_LANGUAGES.includes(request.sourceLanguage.toLowerCase())) {
        return {
          originalText: request.text,
          translatedText: '',
          sourceLanguage: request.sourceLanguage,
          targetLanguage: request.targetLanguage,
          timestamp: Date.now(),
          success: false,
          error: `Source language '${request.sourceLanguage}' is not supported. Supported languages: ${this.SUPPORTED_SOURCE_LANGUAGES.join(', ')}`
        };
      }

      // Validate target language
      if (!this.SUPPORTED_TARGET_LANGUAGES.includes(request.targetLanguage.toLowerCase())) {
        return {
          originalText: request.text,
          translatedText: '',
          sourceLanguage: request.sourceLanguage,
          targetLanguage: request.targetLanguage,
          timestamp: Date.now(),
          success: false,
          error: `Target language '${request.targetLanguage}' is not supported. Supported languages: ${this.SUPPORTED_TARGET_LANGUAGES.join(', ')}`
        };
      }

      // Call the backend translation API
      let translatedText = '';
      try {
        const response = await fetch(`${this.API_BASE_URL}/api/translate`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            text: request.text,
            target_language: request.targetLanguage.toLowerCase().includes('roman') || request.targetLanguage.toLowerCase().includes('urdu') ? 'roman_urdu' : 'english'
          })
        });

        if (!response.ok) {
          throw new Error(`Translation API error: ${response.status} ${response.statusText}`);
        }

        const apiResponse = await response.json();

        if (!apiResponse.success) {
          throw new Error(apiResponse.error || 'Translation API returned error');
        }

        translatedText = apiResponse.translated_text || '';
      } catch (translationError) {
        console.error('Error during translation process:', translationError);
        return {
          originalText: request.text,
          translatedText: '',
          sourceLanguage: request.sourceLanguage,
          targetLanguage: request.targetLanguage,
          timestamp: Date.now(),
          success: false,
          error: `Translation process failed: ${translationError instanceof Error ? translationError.message : 'Unknown error'}`
        };
      }

      // Verify that the translation was successful
      if (typeof translatedText !== 'string') {
        return {
          originalText: request.text,
          translatedText: '',
          sourceLanguage: request.sourceLanguage,
          targetLanguage: request.targetLanguage,
          timestamp: Date.now(),
          success: false,
          error: 'Translation process returned invalid result'
        };
      }

      // Create response object
      const response: TranslationResponse = {
        originalText: request.text,
        translatedText,
        sourceLanguage: request.sourceLanguage,
        targetLanguage: request.targetLanguage,
        timestamp: Date.now(),
        success: true
      };

      // Add to history if translation was successful
      if (response.success && response.translatedText) {
        try {
          const historyItem: TranslationHistoryItem = {
            id: Date.now().toString() + Math.random().toString(36).substr(2, 9),
            originalText: request.text,
            translatedText: response.translatedText,
            timestamp: response.timestamp,
            sourceLanguage: request.sourceLanguage,
            targetLanguage: request.targetLanguage
          };

          const historyAdded = translationHistoryStorage.addItem(historyItem);
          if (!historyAdded) {
            console.warn('Failed to add translation to history, but translation was successful');
          }
        } catch (historyError) {
          console.error('Error adding translation to history:', historyError);
          // Don't fail the translation if history storage fails
        }
      }

      return response;
    } catch (error) {
      console.error('Translation service error:', error);
      return {
        originalText: request.text,
        translatedText: '',
        sourceLanguage: request.sourceLanguage,
        targetLanguage: request.targetLanguage,
        timestamp: Date.now(),
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error occurred during translation'
      };
    }
  }

  async translateBatch(requests: TranslationRequest[]): Promise<TranslationResponse[]> {
    const results: TranslationResponse[] = [];

    for (const request of requests) {
      const result = await this.translate(request);
      results.push(result);
    }

    return results;
  }

  getTranslationHistory(limit?: number): TranslationHistoryItem[] {
    return translationHistoryStorage.getHistory(limit);
  }

  clearTranslationHistory(): boolean {
    return translationHistoryStorage.clearHistory();
  }
}

// Export a singleton instance of the translation service
export const translationService = new TranslationService();

export default translationService;