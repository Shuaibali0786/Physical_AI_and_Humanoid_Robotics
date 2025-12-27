/**
 * Centered Navbar Icons Component
 *
 * This component displays the Translate and Search icons centered in the navbar.
 */

import React, { useState } from 'react';
import IconWrapper from './IconWrapper';
import TranslateButton from '../Translation/TranslateButton';
import SearchButton from '../Search/SearchButton';
import './../../styles/translation-search.css';

interface CenteredNavbarIconsProps {
  onTranslateToggle?: () => void;
  onSearchOpen?: () => void;
  translateActive?: boolean;
  searchActive?: boolean;
}

const CenteredNavbarIcons: React.FC<CenteredNavbarIconsProps> = ({
  onTranslateToggle,
  onSearchOpen,
  translateActive = false,
  searchActive = false
}) => {
  return (
    <div className="navbar-center">
      <TranslateButton
        onClick={onTranslateToggle}
        isActive={translateActive}
        className="navbar-translate-icon"
      />
      <SearchButton
        onClick={onSearchOpen}
        isActive={searchActive}
        className="navbar-search-icon"
      />
    </div>
  );
};

export default CenteredNavbarIcons;