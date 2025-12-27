/**
 * Search Button Component
 *
 * This component provides a button for the search feature
 * that can be placed in the navbar.
 */

import React from 'react';
import IconWrapper from '../Navbar/IconWrapper';
import './../../styles/translation-search.css';

interface SearchButtonProps {
  onClick?: () => void;
  isActive?: boolean;
  className?: string;
}

const SearchButton: React.FC<SearchButtonProps> = ({
  onClick,
  isActive = false,
  className = ''
}) => {
  return (
    <IconWrapper
      onClick={onClick}
      isActive={isActive}
      title="Search Book Modules"
      className={`${className} search-icon`}
    >
      <svg width="18" height="18" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M9.5 3A6.5 6.5 0 0 1 16 9.5c0 1.61-.59 3.09-1.56 4.23l.27.27h.79l5 5-1.5 1.5-5-5v-.79l-.27-.27A6.516 6.516 0 0 1 9.5 16 6.5 6.5 0 0 1 3 9.5 6.5 6.5 0 0 1 9.5 3m0 2C7 5 5 7 5 9.5S7 14 9.5 14 14 12 14 9.5 12 5 9.5 5Z" fill="currentColor"/>
      </svg>
    </IconWrapper>
  );
};

export default SearchButton;