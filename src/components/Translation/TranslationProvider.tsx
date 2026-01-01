/**
 * Translation Provider Context
 *
 * This context provides translation state and functionality
 * to all components in the application that need translation features.
 */

import React, { createContext, useContext, ReactNode, useState, useEffect } from 'react';
import { useTranslation } from '../../hooks/useTranslation';
import { TranslationHistoryItem } from '../../utils/storage';

// Define the context type
interface TranslationContextType {
  isTranslating: boolean;
  isTranslated: boolean;
  translatedText: string;
  originalText: string;
  error: string | null;
  translate: (text: string) => Promise<any>;
  getTranslationHistory: (limit?: number) => TranslationHistoryItem[];
  clearTranslationHistory: () => boolean;
  toggleTranslation: (text: string) => Promise<any>;
  revertToOriginal: () => void;
  currentContent: string;
  setCurrentContent: (content: string) => void;
  translateCurrentContent: () => void;
}

// Create the context with default values
const TranslationContext = createContext<TranslationContextType | undefined>(undefined);

// Props for the provider component
interface TranslationProviderProps {
  children: ReactNode;
  defaultSourceLanguage?: string;
  defaultTargetLanguage?: string;
}

// Translation Provider Component
export const TranslationProvider: React.FC<TranslationProviderProps> = ({
  children,
  defaultSourceLanguage = 'en',
  defaultTargetLanguage = 'roman-urdu'
}) => {
  // Use the translation hook
  const {
    isTranslating,
    isTranslated,
    translatedText,
    originalText,
    error,
    translate,
    getTranslationHistory,
    clearTranslationHistory,
    toggleTranslation,
    revertToOriginal
  } = useTranslation({
    sourceLanguage: defaultSourceLanguage,
    targetLanguage: defaultTargetLanguage
  });

  // State for current content to translate
  const [currentContent, setCurrentContent] = useState<string>('');

  // Function to translate the current content
  const translateCurrentContent = async () => {
    if (currentContent.trim()) {
      await translate(currentContent);
    }
  };

  // Provide context value
  const contextValue: TranslationContextType = {
    isTranslating,
    isTranslated,
    translatedText,
    originalText,
    error,
    translate,
    getTranslationHistory,
    clearTranslationHistory,
    toggleTranslation,
    revertToOriginal,
    currentContent,
    setCurrentContent,
    translateCurrentContent
  };

  return (
    <TranslationContext.Provider value={contextValue}>
      {children}
    </TranslationContext.Provider>
  );
};

// Custom hook to use the translation context
export const useTranslationContext = (): TranslationContextType => {
  const context = useContext(TranslationContext);
  if (context === undefined) {
    throw new Error('useTranslationContext must be used within a TranslationProvider');
  }
  return context;
};

export default TranslationProvider;