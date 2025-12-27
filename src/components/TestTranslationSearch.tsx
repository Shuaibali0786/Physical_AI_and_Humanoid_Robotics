/**
 * Test Component for Translation and Search Features
 *
 * This component tests the integration of translation and search features
 * to ensure they work correctly together.
 */

import React, { useState } from 'react';
import TranslationDisplay from './Translation/TranslationDisplay';
import { useTranslation } from '../hooks/useTranslation';
import { useSearch } from '../hooks/useSearch';
import CustomSearchModal from './Search/CustomSearchModal';
import CenteredNavbarIcons from './Navbar/CenteredNavbarIcons';
import './../../src/styles/translation-search.css';

const TestTranslationSearch = () => {
  const [isSearchModalOpen, setIsSearchModalOpen] = useState(false);
  const [testContent, setTestContent] = useState('This is a sample text to test the translation functionality. Physical AI and Humanoid Robotics are advanced fields.');

  const translation = useTranslation();
  const search = useSearch();

  const handleSearchToggle = () => {
    setIsSearchModalOpen(!isSearchModalOpen);
  };

  const handleTranslateToggle = () => {
    if (translation.isTranslated) {
      translation.revertToOriginal();
    } else {
      translation.translate(testContent);
    }
  };

  return (
    <div className="test-translation-search">
      <h1>Translation and Search Integration Test</h1>

      <div className="test-navbar">
        <CenteredNavbarIcons
          onTranslateToggle={handleTranslateToggle}
          onSearchOpen={handleSearchToggle}
          translateActive={translation.isTranslated}
          searchActive={isSearchModalOpen}
        />
      </div>

      <div className="test-content">
        <h2>Test Translation</h2>
        <textarea
          value={testContent}
          onChange={(e) => setTestContent(e.target.value)}
          className="test-input"
          rows={4}
          cols={60}
        />

        <div className="test-translation-display">
          <TranslationDisplay
            content={testContent}
            showControls={true}
            onTranslationComplete={(isTranslated) => {
              console.log('Translation completed, is translated:', isTranslated);
            }}
          />
        </div>

        <div className="test-status">
          <p><strong>Translation Status:</strong> {translation.isTranslating ? 'Translating...' : translation.isTranslated ? 'Translated' : 'Original'}</p>
          <p><strong>Translation Error:</strong> {translation.error || 'None'}</p>
        </div>
      </div>

      <div className="test-search">
        <h2>Test Search</h2>
        <button onClick={() => setIsSearchModalOpen(true)}>
          Open Search Modal
        </button>
      </div>

      <CustomSearchModal
        isOpen={isSearchModalOpen}
        onClose={() => setIsSearchModalOpen(false)}
      />
    </div>
  );
};

export default TestTranslationSearch;