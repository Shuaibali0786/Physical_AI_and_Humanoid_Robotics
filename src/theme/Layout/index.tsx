/**
 * Custom Layout Component with Error Handling
 *
 * This component wraps the default Docusaurus layout and includes
 * translation and search functionality, with error handling to prevent blank pages.
 */

import React, { useState, useEffect, Suspense, lazy } from 'react';
import Layout from '@theme-original/Layout';
import { useLocation } from '@docusaurus/router';

// Lazy load components to handle potential import errors gracefully
const CenteredNavbarIcons = lazy(() => import('../../components/Navbar/CenteredNavbarIcons').catch(() => {
  console.warn('CenteredNavbarIcons component failed to load');
  return { default: () => null };
}));

const CustomSearchModal = lazy(() => import('../../components/Search/CustomSearchModal').catch(() => {
  console.warn('CustomSearchModal component failed to load');
  return { default: () => null };
}));

const TranslationProvider = lazy(() => import('../../components/Translation/TranslationProvider').catch(() => {
  console.warn('TranslationProvider component failed to load');
  return { default: ({ children }) => <>{children}</> };
}));

const MainChat = lazy(() => import('../../components/MainChat/MainChat').catch(() => {
  console.warn('MainChat component failed to load');
  return { default: () => null };
}));

const ErrorBoundary = ({ children }) => {
  if (typeof window !== 'undefined') {
    // Only run error boundary logic in browser environment
    try {
      return <>{children}</>;
    } catch (error) {
      console.error('Error in layout component:', error);
      return <>{children}</>; // Fallback to children without additional components
    }
  }
  return <>{children}</>;
};

const CustomLayout = (props) => {
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

  // Wrap the layout with error handling
  return (
    <ErrorBoundary>
      <Suspense fallback={<Layout {...props}>{props.children}</Layout>}>
        <TranslationProvider>
          <Layout {...props}>
            {props.children}
            <div
              style={{
                position: 'fixed',
                top: '10px',
                left: '50%',
                transform: 'translateX(-50%)',
                zIndex: 1000,
                display: 'flex',
                justifyContent: 'center',
                alignItems: 'center'
              }}
            >
              <Suspense fallback={null}>
                <CenteredNavbarIcons
                  onTranslateToggle={handleTranslateToggle}
                  onSearchOpen={handleSearchToggle}
                  translateActive={isTranslationActive}
                  searchActive={isSearchModalOpen}
                />
              </Suspense>
            </div>
            {isSearchModalOpen && (
              <Suspense fallback={null}>
                <CustomSearchModal
                  isOpen={isSearchModalOpen}
                  onClose={() => setIsSearchModalOpen(false)}
                />
              </Suspense>
            )}
            <Suspense fallback={null}>
              <MainChat />
            </Suspense>
          </Layout>
        </TranslationProvider>
      </Suspense>
    </ErrorBoundary>
  );
};

export default CustomLayout;