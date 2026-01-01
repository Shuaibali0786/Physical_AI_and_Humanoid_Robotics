# Quickstart Guide: Translation and Search Features

## Overview
This guide will help you set up and start using the new Translation and Search features in the Physical AI & Humanoid Robotics book project. The features include English to Roman Urdu translation and enhanced search functionality.

## Prerequisites
- Node.js 16+ installed
- Yarn or npm package manager
- Docusaurus project setup (already configured in this project)

## Installation

### 1. Install Required Dependencies
```bash
cd E:\physical_ai_text_book\Physical_AI_and_Humanoid_Robotics
npm install
```

### 2. Create Translation Directory
The system will automatically create a `translation` folder for managing translation history:

```bash
mkdir -p src/translation
```

## Setting Up the Translation Feature

### 1. Add Translation Components
Create the necessary components in the `src/components/Translation` directory:

```bash
mkdir -p src/components/Translation
```

### 2. Translation Service Implementation
The translation service uses a client-side transliteration algorithm to convert English text to Roman Urdu. The core functionality is in `src/utils/transliterate.ts`.

### 3. Translation History Management
Translation history is automatically stored in the browser's localStorage under the `translation-history` key.

## Setting Up the Search Feature

### 1. Enhanced Search Components
The search feature extends Docusaurus' built-in Algolia search with custom components in `src/components/Search`.

### 2. Search History
Search history is stored in the browser's localStorage under the `search-history` key.

## Using the Features

### Translation Feature
1. Navigate to any book module page
2. Click the **Translate** icon in the centered navbar
3. The English content will be converted to Roman Urdu format
4. Click the Translate icon again to toggle back to English
5. Translation history is automatically saved and accessible

### Search Feature
1. Click the **Search** icon in the centered navbar
2. Enter your search query in the modal
3. Browse the results which include context and navigation links
4. Search history is automatically maintained

## Configuration

### Environment Variables
Add the following to your `.env` file if needed:
```env
NEXT_PUBLIC_TRANSLATION_ENABLED=true
NEXT_PUBLIC_SEARCH_HISTORY_ENABLED=true
```

### Component Integration
The navbar icons are integrated into the existing Docusaurus theme. They are centered as requested in the requirements.

## Development

### Running the Development Server
```bash
npm run start
```

### Testing the Features
1. Verify translation functionality works on all book modules
2. Test search across different modules
3. Confirm translation history is properly maintained
4. Ensure UI elements are centered and balanced

### File Structure
```
src/
├── components/
│   ├── Translation/
│   │   ├── TranslateButton.tsx
│   │   ├── TranslationProvider.tsx
│   │   ├── TranslationService.ts
│   │   ├── TranslationHistoryManager.ts
│   │   └── TranslationDisplay.tsx
│   ├── Search/
│   │   ├── SearchButton.tsx
│   │   ├── CustomSearchModal.tsx
│   │   └── SearchHistory.tsx
│   └── Navbar/
│       ├── CenteredNavbarIcons.tsx
│       └── IconWrapper.tsx
├── hooks/
│   ├── useTranslation.ts
│   └── useSearch.ts
├── services/
│   ├── translationService.ts
│   └── searchService.ts
├── utils/
│   ├── transliterate.ts
│   └── storage.ts
└── styles/
    └── translation-search.css
```

## API Endpoints
- `POST /api/translate` - Translate English to Roman Urdu
- `GET /api/translation-history` - Get translation history
- `POST /api/translation-history` - Add to translation history
- `GET /api/search` - Search across book modules
- `GET /api/search/history` - Get search history
- `POST /api/search/history` - Add to search history

## Troubleshooting

### Translation Not Working
- Check if the transliteration library is properly loaded
- Verify that the text selection mechanism is working
- Ensure the translation button is properly connected to the translation service

### Search Not Returning Results
- Verify that the search index is properly built
- Check if the search query is being processed correctly
- Confirm that the book modules are properly indexed

### Icons Not Centered
- Check the CSS styling for the navbar components
- Verify that the IconWrapper is properly implemented
- Ensure the Docusaurus theme allows for custom navbar elements

## Next Steps
1. Implement the TranslationProvider context
2. Create the TranslateButton component with toggle functionality
3. Build the SearchButton and CustomSearchModal components
4. Integrate with the existing RAG chatbot infrastructure
5. Test the features across all book modules