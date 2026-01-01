/**
 * Translation History Manager Service
 *
 * This service manages the translation history, providing functionality
 * to save, retrieve, clear, and manage translation history items.
 */

import { TranslationHistoryItem } from '../../utils/storage';

interface TranslationHistoryManagerOptions {
  maxHistoryItems?: number;
  autoSave?: boolean;
}

export class TranslationHistoryManager {
  private maxHistoryItems: number;
  private autoSave: boolean;

  constructor(options: TranslationHistoryManagerOptions = {}) {
    this.maxHistoryItems = options.maxHistoryItems || 100;
    this.autoSave = options.autoSave !== undefined ? options.autoSave : true;
  }

  /**
   * Adds a translation to history
   * @param originalText The original English text
   * @param translatedText The translated Roman Urdu text
   * @returns Boolean indicating success
   */
  async addTranslation(originalText: string, translatedText: string): Promise<boolean> {
    try {
      // Import the storage utility dynamically to avoid circular dependencies
      const { translationHistoryStorage, TranslationHistoryItem } = await import('../../utils/storage');

      const historyItem: TranslationHistoryItem = {
        id: Date.now().toString() + Math.random().toString(36).substr(2, 9),
        originalText,
        translatedText,
        timestamp: Date.now(),
        sourceLanguage: 'en',
        targetLanguage: 'roman-urdu'
      };

      return translationHistoryStorage.addItem(historyItem);
    } catch (error) {
      console.error('Error adding translation to history:', error);
      return false;
    }
  }

  /**
   * Gets the translation history
   * @param limit Optional limit on number of items to return
   * @returns Array of translation history items
   */
  async getHistory(limit?: number): Promise<TranslationHistoryItem[]> {
    try {
      const { translationHistoryStorage } = await import('../../utils/storage');
      return translationHistoryStorage.getHistory(limit);
    } catch (error) {
      console.error('Error getting translation history:', error);
      return [];
    }
  }

  /**
   * Clears the translation history
   * @returns Boolean indicating success
   */
  async clearHistory(): Promise<boolean> {
    try {
      const { translationHistoryStorage } = await import('../../utils/storage');
      return translationHistoryStorage.clearHistory();
    } catch (error) {
      console.error('Error clearing translation history:', error);
      return false;
    }
  }

  /**
   * Removes a specific translation from history
   * @param id The ID of the translation to remove
   * @returns Boolean indicating success
   */
  async removeTranslation(id: string): Promise<boolean> {
    try {
      const { translationHistoryStorage } = await import('../../utils/storage');
      return translationHistoryStorage.removeItem(id);
    } catch (error) {
      console.error('Error removing translation from history:', error);
      return false;
    }
  }

  /**
   * Gets the count of translation history items
   * @returns Number of items in history
   */
  async getHistoryCount(): Promise<number> {
    try {
      const history = await this.getHistory();
      return history.length;
    } catch (error) {
      console.error('Error getting translation history count:', error);
      return 0;
    }
  }

  /**
   * Updates a translation in history
   * @param id The ID of the translation to update
   * @param updates Partial updates to apply to the translation
   * @returns Boolean indicating success
   */
  async updateTranslation(id: string, updates: Partial<TranslationHistoryItem>): Promise<boolean> {
    try {
      // First, get the current history
      const { translationHistoryStorage } = await import('../../utils/storage');
      const history = translationHistoryStorage.getHistory();

      // Find the item to update
      const itemIndex = history.findIndex(item => item.id === id);
      if (itemIndex === -1) {
        return false;
      }

      // Update the item
      const updatedItem = {
        ...history[itemIndex],
        ...updates
      };

      // Remove the old item
      const filteredHistory = history.filter(item => item.id !== id);

      // Add the updated item
      const updatedHistory = [updatedItem, ...filteredHistory];

      // Save the updated history
      const historyJson = JSON.stringify(updatedHistory);
      return translationHistoryStorage.setItem('translation-history', historyJson);
    } catch (error) {
      console.error('Error updating translation in history:', error);
      return false;
    }
  }

  /**
   * Searches translation history for specific text
   * @param searchTerm The text to search for in original or translated text
   * @param limit Optional limit on number of items to return
   * @returns Array of matching translation history items
   */
  async searchHistory(searchTerm: string, limit?: number): Promise<TranslationHistoryItem[]> {
    try {
      const history = await this.getHistory();
      const searchLower = searchTerm.toLowerCase();

      const matches = history.filter(item =>
        item.originalText.toLowerCase().includes(searchLower) ||
        item.translatedText.toLowerCase().includes(searchLower)
      );

      return limit ? matches.slice(0, limit) : matches;
    } catch (error) {
      console.error('Error searching translation history:', error);
      return [];
    }
  }

  /**
   * Exports the translation history as JSON string
   * @returns JSON string of the translation history
   */
  async exportHistory(): Promise<string> {
    try {
      const history = await this.getHistory();
      return JSON.stringify(history, null, 2);
    } catch (error) {
      console.error('Error exporting translation history:', error);
      return JSON.stringify([]);
    }
  }

  /**
   * Imports translation history from JSON string
   * @param jsonString The JSON string to import
   * @returns Boolean indicating success
   */
  async importHistory(jsonString: string): Promise<boolean> {
    try {
      const { translationHistoryStorage } = await import('../../utils/storage');

      // Validate the JSON string
      const parsed = JSON.parse(jsonString);

      if (!Array.isArray(parsed)) {
        throw new Error('Invalid history format: expected array');
      }

      // Validate each item in the array
      for (const item of parsed) {
        if (
          typeof item.id !== 'string' ||
          typeof item.originalText !== 'string' ||
          typeof item.translatedText !== 'string' ||
          typeof item.timestamp !== 'number' ||
          typeof item.sourceLanguage !== 'string' ||
          typeof item.targetLanguage !== 'string'
        ) {
          throw new Error('Invalid history item format');
        }
      }

      // Limit the number of items to import
      const limitedHistory = parsed.slice(0, this.maxHistoryItems);

      // Save the imported history
      const historyJson = JSON.stringify(limitedHistory);
      return translationHistoryStorage.setItem('translation-history', historyJson);
    } catch (error) {
      console.error('Error importing translation history:', error);
      return false;
    }
  }
}

// Export a singleton instance of the TranslationHistoryManager
export const translationHistoryManager = new TranslationHistoryManager();

export default TranslationHistoryManager;