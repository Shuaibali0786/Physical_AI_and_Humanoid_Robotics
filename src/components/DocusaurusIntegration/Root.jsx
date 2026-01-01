/**
 * Docusaurus Root Component for Translation and Search Integration
 *
 * This component wraps the entire application to provide
 * translation and search functionality across all pages.
 */

import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import CenteredNavbarIcons from '../Navbar/CenteredNavbarIcons';
import CustomSearchModal from '../Search/CustomSearchModal';
import TranslationProvider from '../Translation/TranslationProvider';

const Root = ({ children }) => {
  const location = useLocation();
  const [isSearchModalOpen, setIsSearchModalOpen] = useState(false);
  const [isTranslationActive, setIsTranslationActive] = useState(false);

  // Close modals when route changes
  useEffect(() => {
    setIsSearchModalOpen(false);
  }, [location]);

  const handleSearchToggle = () => {
    setIsSearchModalOpen(!isSearchModalOpen);
  };

  const handleTranslateToggle = () => {
    setIsTranslationActive(!isTranslationActive);
  };

  return (
    <TranslationProvider>
      <div style={{ position: 'relative' }}>
        {/* Centered navbar icons - positioned fixed at top center */}
        <div
          style={{
            position: 'fixed',
            top: '10px',
            left: '50%',
            transform: 'translateX(-50%)',
            zIndex: 1000,
            display: 'flex',
            justifyContent: 'center',
            alignItems: 'center',
            backgroundColor: 'var(--ifm-navbar-background-color, #fff)',
            borderRadius: '8px',
            padding: '4px',
            boxShadow: '0 2px 8px rgba(0,0,0,0.1)'
          }}
        >
          <CenteredNavbarIcons
            onTranslateToggle={handleTranslateToggle}
            onSearchOpen={handleSearchToggle}
            translateActive={isTranslationActive}
            searchActive={isSearchModalOpen}
          />
        </div>

        {children}

        <CustomSearchModal
          isOpen={isSearchModalOpen}
          onClose={() => setIsSearchModalOpen(false)}
        />
      </div>
    </TranslationProvider>
  );
};

export default Root;