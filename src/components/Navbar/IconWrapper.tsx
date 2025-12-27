/**
 * Icon Wrapper Component
 *
 * This component wraps icons for the navbar with consistent styling
 * and behavior for the translation and search features.
 */

import React from 'react';
import './../../styles/translation-search.css';

interface IconWrapperProps {
  children: React.ReactNode;
  onClick?: () => void;
  isActive?: boolean;
  title?: string;
  className?: string;
  disabled?: boolean;
}

const IconWrapper: React.FC<IconWrapperProps> = ({
  children,
  onClick,
  isActive = false,
  title,
  className = '',
  disabled = false
}) => {
  const handleClick = () => {
    if (!disabled && onClick) {
      onClick();
    }
  };

  const iconClasses = [
    'navbar-icon',
    className,
    isActive ? 'active' : '',
    disabled ? 'disabled' : ''
  ].filter(Boolean).join(' ');

  return (
    <div
      className={iconClasses}
      onClick={handleClick}
      title={title}
      role="button"
      tabIndex={disabled ? -1 : 0}
      onKeyDown={(e) => {
        if (e.key === 'Enter' && !disabled) {
          handleClick();
        }
      }}
    >
      {children}
      {title && (
        <span className="navbar-icon-tooltip">{title}</span>
      )}
    </div>
  );
};

export default IconWrapper;