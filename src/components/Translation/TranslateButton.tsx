/**
 * Translate Button Component
 *
 * This component provides a button for the translation feature
 * that can be placed in the navbar.
 */

import React from 'react';
import IconWrapper from '../Navbar/IconWrapper';
import './../../styles/translation-search.css';

interface TranslateButtonProps {
  onClick?: () => void;
  isActive?: boolean;
  className?: string;
}

const TranslateButton: React.FC<TranslateButtonProps> = ({
  onClick,
  isActive = false,
  className = ''
}) => {
  return (
    <IconWrapper
      onClick={onClick}
      isActive={isActive}
      title={isActive ? 'Switch to English' : 'Translate to Roman Urdu'}
      className={`${className} translate-icon`}
    >
      <svg width="18" height="18" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M12.87 15.07L10.33 12.56L10.36 12.53C12.1 10.59 13.34 8.36 14.07 6H17V4H10V2H8V4H1V6H12.17C11.5 7.92 10.44 9.67 9 11.22L6.61 8.82L5.19 10.24L8.66 13.71L8.68 13.71L12.87 15.07M15.5 16L16.63 14.87L15.5 13.75L14.37 14.87L15.5 16M18.5 13L19.63 11.87L18.5 10.75L17.37 11.87L18.5 13M20.71 5.63L22.12 7.04L12 17.17L8.12 13.29L9.54 11.87L12 14.33L20.71 5.63Z" fill="currentColor"/>
      </svg>
    </IconWrapper>
  );
};

export default TranslateButton;