/**
 * Core Translation Logic
 *
 * This module contains the core logic for translating content,
 * including validation, processing, and integration with the translation service.
 */

import { englishToRomanUrdu } from '../../utils/transliterate';
import { translationService, TranslationRequest, TranslationResponse } from '../../services/translationService';
import { translationHistoryManager } from './TranslationHistoryManager';

export interface TranslationOptions {
  validateInput?: boolean;
  preserveFormatting?: boolean;
  addToHistory?: boolean;
}

export interface ProcessedTranslationResult {
  success: boolean;
  originalText: string;
  translatedText: string;
  error?: string;
  processingTime: number;
}

/**
 * Validates the input text for translation
 * @param text The text to validate
 * @returns Boolean indicating if the text is valid for translation
 */
export function validateTranslationInput(text: string): boolean {
  if (typeof text !== 'string') {
    return false;
  }

  if (text.trim().length === 0) {
    return false;
  }

  // Check for maximum length to prevent performance issues
  if (text.length > 10000) { // 10,000 characters max
    return false;
  }

  return true;
}

/**
 * Processes a single text for translation
 * @param text The text to translate
 * @param options Translation options
 * @returns Processed translation result
 */
export async function processTranslation(text: string, options: TranslationOptions = {}): Promise<ProcessedTranslationResult> {
  const {
    validateInput = true,
    preserveFormatting = false,
    addToHistory = true
  } = options;

  const startTime = Date.now();

  try {
    // Validate input if required
    if (validateInput && !validateTranslationInput(text)) {
      return {
        success: false,
        originalText: text,
        translatedText: '',
        error: 'Invalid input text for translation',
        processingTime: Date.now() - startTime
      };
    }

    // Prepare the translation request
    const request: TranslationRequest = {
      text,
      sourceLanguage: 'en',
      targetLanguage: 'roman-urdu',
      preserveFormatting
    };

    // Perform the translation using the service
    const response: TranslationResponse = await translationService.translate(request);

    // Add to history if successful and option is enabled
    if (response.success && addToHistory && response.translatedText) {
      await translationHistoryManager.addTranslation(response.originalText, response.translatedText);
    }

    return {
      success: response.success,
      originalText: response.originalText,
      translatedText: response.translatedText,
      error: response.error,
      processingTime: Date.now() - startTime
    };
  } catch (error) {
    return {
      success: false,
      originalText: text,
      translatedText: '',
      error: error instanceof Error ? error.message : 'Unknown error occurred during translation',
      processingTime: Date.now() - startTime
    };
  }
}

/**
 * Processes multiple texts for translation
 * @param texts Array of texts to translate
 * @param options Translation options
 * @returns Array of processed translation results
 */
export async function processMultipleTranslations(texts: string[], options: TranslationOptions = {}): Promise<ProcessedTranslationResult[]> {
  const results: ProcessedTranslationResult[] = [];

  for (const text of texts) {
    const result = await processTranslation(text, options);
    results.push(result);
  }

  return results;
}

/**
 * Translates content while preserving HTML-like structure
 * @param content The content that may contain HTML-like structure
 * @param options Translation options
 * @returns Translated content with preserved structure
 */
export async function translateWithStructurePreservation(content: string, options: TranslationOptions = {}): Promise<ProcessedTranslationResult> {
  const {
    preserveFormatting = true,
    addToHistory = true
  } = options;

  const startTime = Date.now();

  try {
    // This is a simplified approach to preserve structure
    // In a more complex implementation, you might want to parse HTML
    // and only translate text nodes while preserving tags

    // For now, we'll just translate the entire content
    // and if preserveFormatting is true, we'll maintain the original structure
    const result = await processTranslation(content, { ...options, preserveFormatting });

    if (result.success && addToHistory) {
      await translationHistoryManager.addTranslation(result.originalText, result.translatedText);
    }

    return {
      ...result,
      processingTime: Date.now() - startTime
    };
  } catch (error) {
    return {
      success: false,
      originalText: content,
      translatedText: '',
      error: error instanceof Error ? error.message : 'Unknown error occurred during structured translation',
      processingTime: Date.now() - startTime
    };
  }
}

/**
 * Translates content by paragraphs
 * @param content The content to translate (multiple paragraphs)
 * @param options Translation options
 * @returns Translated content with paragraph preservation
 */
export async function translateByParagraphs(content: string, options: TranslationOptions = {}): Promise<ProcessedTranslationResult> {
  const startTime = Date.now();

  try {
    // Split content by paragraphs (double newlines)
    const paragraphs = content.split(/\n\s*\n/);

    // Translate each paragraph
    const translatedParagraphs = [];
    let overallSuccess = true;
    let overallError: string | undefined;

    for (const paragraph of paragraphs) {
      if (paragraph.trim() === '') {
        // Keep empty paragraphs as they are
        translatedParagraphs.push('');
        continue;
      }

      const result = await processTranslation(paragraph, options);

      if (result.success) {
        translatedParagraphs.push(result.translatedText);
      } else {
        overallSuccess = false;
        if (result.error) {
          overallError = result.error;
        }
        // Add the original paragraph if translation failed
        translatedParagraphs.push(paragraph);
      }
    }

    const translatedContent = translatedParagraphs.join('\n\n');

    // Add to history if successful
    if (overallSuccess && options.addToHistory) {
      await translationHistoryManager.addTranslation(content, translatedContent);
    }

    return {
      success: overallSuccess,
      originalText: content,
      translatedText: translatedContent,
      error: overallError,
      processingTime: Date.now() - startTime
    };
  } catch (error) {
    return {
      success: false,
      originalText: content,
      translatedText: '',
      error: error instanceof Error ? error.message : 'Unknown error occurred during paragraph translation',
      processingTime: Date.now() - startTime
    };
  }
}

/**
 * Gets translation statistics
 * @returns Object containing translation statistics
 */
export async function getTranslationStats(): Promise<{
  totalTranslations: number;
  avgProcessingTime: number;
  successRate: number;
}> {
  try {
    const history = await translationHistoryManager.getHistory();
    const totalTranslations = history.length;

    return {
      totalTranslations,
      avgProcessingTime: 0, // In a real implementation, you'd calculate this from stored processing times
      successRate: 100 // Assuming all stored translations were successful
    };
  } catch (error) {
    console.error('Error getting translation stats:', error);
    return {
      totalTranslations: 0,
      avgProcessingTime: 0,
      successRate: 0
    };
  }
}