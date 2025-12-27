/**
 * Custom Navbar Component with Translation and Search Icons
 *
 * This component extends the default Docusaurus navbar to include
 * centered translation and search icons as specified in the requirements.
 */

import React, { useState } from 'react';
import Navbar from '@theme-original/Navbar';
import CenteredNavbarIcons from '../../components/Navbar/CenteredNavbarIcons';
import CustomSearchModal from '../../components/Search/CustomSearchModal';
import TranslationModuleWrapper from '../../components/Translation/TranslationModuleWrapper';
import TranslationProvider from '../../components/Translation/TranslationProvider';

const CustomNavbar = (props) => {
  const [isSearchModalOpen, setIsSearchModalOpen] = useState(false);
  const [isTranslationActive, setIsTranslationActive] = useState(false);

  const handleSearchToggle = () => {
    setIsSearchModalOpen(!isSearchModalOpen);
  };

  const handleTranslateToggle = () => {
    setIsTranslationActive(!isTranslationActive);
  };

  return (
    <>
      <Navbar {...props} />
      <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '0' }}>
        <CenteredNavbarIcons
          onTranslateToggle={handleTranslateToggle}
          onSearchOpen={handleSearchToggle}
          translateActive={isTranslationActive}
          searchActive={isSearchModalOpen}
        />
      </div>
      <CustomSearchModal
        isOpen={isSearchModalOpen}
        onClose={() => setIsSearchModalOpen(false)}
      />
      <TranslationProvider>
        {/* The translation functionality will be available throughout the app */}
      </TranslationProvider>
    </>
  );
};

export default CustomNavbar;