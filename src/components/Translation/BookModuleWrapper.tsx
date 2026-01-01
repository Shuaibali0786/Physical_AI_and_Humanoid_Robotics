/**
 * Book Module Wrapper Component
 *
 * This component wraps book module content to provide translation functionality
 * while maintaining the original Docusaurus content structure.
 */

import React from 'react';
import TranslationDisplay from './TranslationDisplay';
import TranslationProvider from './TranslationProvider';
import './../../styles/translation-search.css';

interface BookModuleWrapperProps {
  children: React.ReactNode;
  title?: string;
}

const BookModuleWrapper: React.FC<BookModuleWrapperProps> = ({
  children,
  title = 'Book Module'
}) => {
  // For now, we'll just return the children wrapped in the translation provider
  // In a more advanced implementation, we could extract and translate the content
  return (
    <TranslationProvider>
      <div className="book-module-wrapper">
        {children}
      </div>
    </TranslationProvider>
  );
};

export default BookModuleWrapper;