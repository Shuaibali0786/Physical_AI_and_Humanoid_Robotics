/**
 * Storage Utility for Translation and Search Features
 *
 * This utility provides a consistent interface for managing browser storage
 * including localStorage and sessionStorage with fallbacks and error handling.
 */

interface StorageOptions {
  useSessionStorage?: boolean;
  fallbackToMemory?: boolean;
}

interface MemoryStorage {
  [key: string]: string | null;
}

class StorageManager {
  private memoryStorage: MemoryStorage = {};
  private storageAvailable: boolean = false;

  constructor() {
    this.storageAvailable = this.checkStorageAvailability();
  }

  private checkStorageAvailability(): boolean {
    try {
      const testKey = '__storage_test__';
      localStorage.setItem(testKey, 'test');
      localStorage.removeItem(testKey);
      return true;
    } catch (e) {
      return false;
    }
  }

  private getStorage(type: 'local' | 'session'): Storage | null {
    if (!this.storageAvailable) {
      return null;
    }

    return type === 'local' ? localStorage : sessionStorage;
  }

  /**
   * Get an item from storage
   * @param key The key to retrieve
   * @param options Storage options
   * @returns The value or null if not found
   */
  getItem(key: string, options: StorageOptions = {}): string | null {
    const { useSessionStorage = false, fallbackToMemory = true } = options;
    const storageType = useSessionStorage ? 'session' : 'local';

    try {
      const storage = this.getStorage(storageType);
      if (storage) {
        return storage.getItem(key);
      }
    } catch (e) {
      console.warn(`Failed to get item from ${storageType} storage:`, e);
    }

    // Fallback to memory storage if available and requested
    if (fallbackToMemory) {
      return this.memoryStorage[key] || null;
    }

    return null;
  }

  /**
   * Set an item in storage
   * @param key The key to set
   * @param value The value to store
   * @param options Storage options
   * @returns Boolean indicating success
   */
  setItem(key: string, value: string, options: StorageOptions = {}): boolean {
    const { useSessionStorage = false, fallbackToMemory = true } = options;
    const storageType = useSessionStorage ? 'session' : 'local';

    try {
      const storage = this.getStorage(storageType);
      if (storage) {
        storage.setItem(key, value);
        return true;
      }
    } catch (e) {
      console.warn(`Failed to set item in ${storageType} storage:`, e);
    }

    // Fallback to memory storage if available and requested
    if (fallbackToMemory) {
      this.memoryStorage[key] = value;
      return true;
    }

    return false;
  }

  /**
   * Remove an item from storage
   * @param key The key to remove
   * @param options Storage options
   * @returns Boolean indicating success
   */
  removeItem(key: string, options: StorageOptions = {}): boolean {
    const { useSessionStorage = false, fallbackToMemory = true } = options;
    const storageType = useSessionStorage ? 'session' : 'local';

    try {
      const storage = this.getStorage(storageType);
      if (storage) {
        storage.removeItem(key);
        return true;
      }
    } catch (e) {
      console.warn(`Failed to remove item from ${storageType} storage:`, e);
    }

    // Fallback to memory storage if available and requested
    if (fallbackToMemory && this.memoryStorage.hasOwnProperty(key)) {
      delete this.memoryStorage[key];
      return true;
    }

    return false;
  }

  /**
   * Clear all items from storage
   * @param options Storage options
   * @returns Boolean indicating success
   */
  clear(options: StorageOptions = {}): boolean {
    const { useSessionStorage = false, fallbackToMemory = true } = options;
    const storageType = useSessionStorage ? 'session' : 'local';

    try {
      const storage = this.getStorage(storageType);
      if (storage) {
        storage.clear();
        return true;
      }
    } catch (e) {
      console.warn(`Failed to clear ${storageType} storage:`, e);
    }

    // Fallback to memory storage if available and requested
    if (fallbackToMemory) {
      this.memoryStorage = {};
      return true;
    }

    return false;
  }

  /**
   * Get all keys from storage
   * @param options Storage options
   * @returns Array of keys
   */
  getKeys(options: StorageOptions = {}): string[] {
    const { useSessionStorage = false, fallbackToMemory = true } = options;
    const storageType = useSessionStorage ? 'session' : 'local';

    try {
      const storage = this.getStorage(storageType);
      if (storage) {
        return Object.keys(storage);
      }
    } catch (e) {
      console.warn(`Failed to get keys from ${storageType} storage:`, e);
    }

    // Fallback to memory storage if available and requested
    if (fallbackToMemory) {
      return Object.keys(this.memoryStorage);
    }

    return [];
  }

  /**
   * Check if storage is available
   * @returns Boolean indicating if storage is available
   */
  isStorageAvailable(): boolean {
    return this.storageAvailable;
  }
}

// Create a singleton instance
const storageManager = new StorageManager();

/**
 * Translation History Storage Interface
 * Manages translation history with automatic cleanup
 */
export interface TranslationHistoryItem {
  id: string;
  originalText: string;
  translatedText: string;
  timestamp: number;
  sourceLanguage: string;
  targetLanguage: string;
}

export interface TranslationHistoryStorage {
  addItem(item: TranslationHistoryItem): boolean;
  getHistory(limit?: number): TranslationHistoryItem[];
  clearHistory(): boolean;
  removeItem(id: string): boolean;
}

/**
 * Implementation of Translation History Storage
 */
export class TranslationHistoryStorageImpl implements TranslationHistoryStorage {
  private readonly STORAGE_KEY = 'translation-history';
  private readonly MAX_HISTORY_ITEMS = 100; // Limit history to prevent storage overflow

  addItem(item: TranslationHistoryItem): boolean {
    try {
      const history = this.getHistory();

      // Add new item to the beginning of the array
      history.unshift(item);

      // Limit the history to MAX_HISTORY_ITEMS
      if (history.length > this.MAX_HISTORY_ITEMS) {
        history.splice(this.MAX_HISTORY_ITEMS);
      }

      // Save updated history
      const historyJson = JSON.stringify(history);
      return storageManager.setItem(this.STORAGE_KEY, historyJson);
    } catch (e) {
      console.error('Failed to add translation history item:', e);
      return false;
    }
  }

  getHistory(limit?: number): TranslationHistoryItem[] {
    try {
      const historyJson = storageManager.getItem(this.STORAGE_KEY);
      if (!historyJson) {
        return [];
      }

      const history: TranslationHistoryItem[] = JSON.parse(historyJson);

      // Return limited results if specified
      if (limit !== undefined) {
        return history.slice(0, limit);
      }

      return history;
    } catch (e) {
      console.error('Failed to get translation history:', e);
      return [];
    }
  }

  clearHistory(): boolean {
    try {
      return storageManager.removeItem(this.STORAGE_KEY);
    } catch (e) {
      console.error('Failed to clear translation history:', e);
      return false;
    }
  }

  removeItem(id: string): boolean {
    try {
      const history = this.getHistory();
      const filteredHistory = history.filter(item => item.id !== id);

      const historyJson = JSON.stringify(filteredHistory);
      return storageManager.setItem(this.STORAGE_KEY, historyJson);
    } catch (e) {
      console.error('Failed to remove translation history item:', e);
      return false;
    }
  }
}

/**
 * Search History Storage Interface
 * Manages search history with automatic cleanup
 */
export interface SearchHistoryItem {
  id: string;
  query: string;
  timestamp: number;
  resultsCount?: number;
}

export interface SearchHistoryStorage {
  addSearch(query: string, resultsCount?: number): boolean;
  getSearchHistory(limit?: number): SearchHistoryItem[];
  clearSearchHistory(): boolean;
  removeSearch(id: string): boolean;
}

/**
 * Implementation of Search History Storage
 */
export class SearchHistoryStorageImpl implements SearchHistoryStorage {
  private readonly STORAGE_KEY = 'search-history';
  private readonly MAX_HISTORY_ITEMS = 50; // Limit search history

  addSearch(query: string, resultsCount?: number): boolean {
    try {
      const history = this.getSearchHistory();

      // Create new search item
      const searchItem: SearchHistoryItem = {
        id: Date.now().toString() + Math.random().toString(36).substr(2, 9), // Generate unique ID
        query,
        timestamp: Date.now(),
        resultsCount
      };

      // Add new item to the beginning of the array
      history.unshift(searchItem);

      // Limit the history to MAX_HISTORY_ITEMS
      if (history.length > this.MAX_HISTORY_ITEMS) {
        history.splice(this.MAX_HISTORY_ITEMS);
      }

      // Save updated history
      const historyJson = JSON.stringify(history);
      return storageManager.setItem(this.STORAGE_KEY, historyJson);
    } catch (e) {
      console.error('Failed to add search history item:', e);
      return false;
    }
  }

  getSearchHistory(limit?: number): SearchHistoryItem[] {
    try {
      const historyJson = storageManager.getItem(this.STORAGE_KEY);
      if (!historyJson) {
        return [];
      }

      const history: SearchHistoryItem[] = JSON.parse(historyJson);

      // Return limited results if specified
      if (limit !== undefined) {
        return history.slice(0, limit);
      }

      return history;
    } catch (e) {
      console.error('Failed to get search history:', e);
      return [];
    }
  }

  clearSearchHistory(): boolean {
    try {
      return storageManager.removeItem(this.STORAGE_KEY);
    } catch (e) {
      console.error('Failed to clear search history:', e);
      return false;
    }
  }

  removeSearch(id: string): boolean {
    try {
      const history = this.getSearchHistory();
      const filteredHistory = history.filter(item => item.id !== id);

      const historyJson = JSON.stringify(filteredHistory);
      return storageManager.setItem(this.STORAGE_KEY, historyJson);
    } catch (e) {
      console.error('Failed to remove search history item:', e);
      return false;
    }
  }
}

// Export the main storage manager and history storage implementations
export default storageManager;
export const translationHistoryStorage = new TranslationHistoryStorageImpl();
export const searchHistoryStorage = new SearchHistoryStorageImpl();