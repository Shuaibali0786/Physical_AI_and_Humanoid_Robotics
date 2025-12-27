/**
 * Translation Wrapper for Book Modules
 *
 * This component wraps around book module content to provide translation functionality
 * that can translate the content from English to Roman Urdu.
 */

import React, { useEffect, useState } from 'react';
import TranslationDisplay from '../Translation/TranslationDisplay';
import { useTranslationContext } from '../Translation/TranslationProvider';
import './../../styles/translation-search.css';

interface TranslationModuleWrapperProps {
  children: React.ReactNode;
  moduleId?: string;
  title?: string;
}

const TranslationModuleWrapper: React.FC<TranslationModuleWrapperProps> = ({
  children,
  moduleId,
  title
}) => {
  const { currentContent, setCurrentContent } = useTranslationContext();
  const [moduleContent, setModuleContent] = useState<string>('');

  // Extract text content from children to translate
  useEffect(() => {
    if (children) {
      // Convert children to string to extract text content
      const extractText = (child: React.ReactNode): string => {
        if (typeof child === 'string') {
          return child;
        } else if (React.isValidElement(child)) {
          if (child.props.children) {
            return extractText(child.props.children);
          }
          return '';
        } else if (Array.isArray(child)) {
          return child.map(extractText).join(' ');
        }
        return '';
      };

      const textContent = extractText(children);
      setModuleContent(textContent);
      setCurrentContent(textContent);
    }
  }, [children, setCurrentContent]);

  // If we want to translate the entire module content
  return (
    <div className="translation-module-wrapper">
      <TranslationDisplay
        content={moduleContent}
        showControls={true}
        preserveFormatting={true}
      >
        {children}
      </TranslationDisplay>
    </div>
  );
};

export default TranslationModuleWrapper;