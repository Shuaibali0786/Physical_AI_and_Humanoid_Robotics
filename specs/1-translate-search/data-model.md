# Data Model: Translate and Search Features

## Translation Request Entity
- **id**: string (UUID) - Unique identifier for the translation request
- **originalText**: string - The original English text to be translated
- **translatedText**: string - The Roman Urdu translation of the text
- **timestamp**: datetime - When the translation was performed
- **sourceModule**: string - The book module where translation was requested
- **status**: enum ['pending', 'completed', 'error'] - Status of the translation

## Translation History Entity
- **historyId**: string (UUID) - Unique identifier for the history entry
- **requests**: array of TranslationRequest - Collection of translation requests
- **lastAccessed**: datetime - Timestamp of last access to this history
- **maxEntries**: number (default: 100) - Maximum number of entries to retain
- **userId**: string (optional) - User identifier if implemented with user accounts

## Search Query Entity
- **queryId**: string (UUID) - Unique identifier for the search query
- **queryString**: string - The search term(s) entered by the user
- **timestamp**: datetime - When the search was performed
- **resultsCount**: number - Number of results returned
- **sourceModule**: string - Module where search was initiated
- **searchType**: enum ['full-text', 'title', 'tag'] - Type of search performed

## Search Result Entity
- **resultId**: string (UUID) - Unique identifier for the search result
- **title**: string - Title of the matching content
- **content**: string - Excerpt of the matching content
- **url**: string - URL to the content location
- **module**: string - Book module containing the content
- **relevance**: number (0-1) - Relevance score of the result
- **queryId**: string - Reference to the search query that generated this result

## Navbar Icon Entity
- **iconId**: string - Unique identifier for the icon
- **name**: enum ['translate', 'search'] - Name/type of the icon
- **label**: string - Accessible label for the icon
- **position**: string (default: 'center') - Positioning in the navbar
- **isActive**: boolean - Whether the icon's feature is currently active
- **tooltip**: string - Tooltip text when hovering over the icon

## User Preferences Entity
- **userId**: string (UUID) - Unique identifier for the user
- **translationEnabled**: boolean (default: false) - Whether translation is currently enabled
- **searchHistoryEnabled**: boolean (default: true) - Whether to maintain search history
- **translationHistoryEnabled**: boolean (default: true) - Whether to maintain translation history
- **lastUsedLanguage**: string (default: 'roman-urdu') - Last target language used
- **fontSizePreference**: number (default: 16) - Preferred font size for translated content

## Validation Rules

### Translation Request Validation
- originalText must be between 1 and 10000 characters
- timestamp must be in ISO 8601 format
- sourceModule must reference an existing book module
- status must be one of the defined enum values

### Translation History Validation
- requests array must not exceed maxEntries limit
- Each request in requests array must be a valid TranslationRequest
- historyId must be unique across all history entries

### Search Query Validation
- queryString must be between 1 and 255 characters
- resultsCount must be a non-negative integer
- searchType must be one of the defined enum values

### Search Result Validation
- relevance score must be between 0 and 1
- url must be a valid relative path to book content
- resultId must be unique within the context of a single query

### Navbar Icon Validation
- name must be one of the defined enum values
- position must be a valid CSS position value
- isActive must be a boolean value

## State Transitions

### Translation Request States
1. **pending** → **completed**: Translation successfully processed
2. **pending** → **error**: Translation failed due to processing error
3. **completed** → **pending**: Translation is being reprocessed

### Search Query States
1. **created** → **processed**: Search has been executed and results returned
2. **processed** → **updated**: New results added to existing query results

## Relationships

### Translation Request → Translation History
- One-to-many: A TranslationHistory can contain many TranslationRequests
- TranslationHistory.requests[] contains multiple TranslationRequest objects

### Search Query → Search Result
- One-to-many: A SearchQuery can generate many SearchResult entries
- SearchQuery includes SearchResult[] array

### User Preferences → Translation History
- One-to-many: UserPreferences can reference multiple TranslationHistory entries
- Used when implementing user-specific history management