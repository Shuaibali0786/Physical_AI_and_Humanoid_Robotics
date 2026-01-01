# Claude Context for Physical AI & Humanoid Robotics Project

## Project Overview
The Physical AI & Humanoid Robotics project is a comprehensive book/resource on AI and robotics, featuring ROS 2, Gazebo, NVIDIA Isaac, and other technologies. The project uses Docusaurus for documentation and includes a RAG chatbot for interactive learning.

## Current Features
- Docusaurus-based documentation system
- RAG chatbot with backend integration
- Book modules covering various topics in AI and robotics
- Interactive learning components

## New Features Being Implemented
### Translation Feature (English → Roman Urdu)
- Client-side transliteration algorithm for English to Roman Urdu conversion
- Translation history management in browser storage
- Toggle functionality to switch between English and Roman Urdu
- Integration with book module content

### Search Enhancement Feature
- Enhanced search functionality beyond Docusaurus' built-in search
- Search history tracking
- Context-aware search results with module information
- Improved search UI with modal interface

## Technical Implementation Details
### Translation System
- Uses rule-based transliteration for client-side conversion
- Stores translation history in browser's localStorage
- Preserves HTML formatting during translation
- Maintains original content when translation is toggled off

### Search System
- Extends Docusaurus' Algolia search with custom components
- Tracks search history for user convenience
- Provides context-aware results with module navigation
- Maintains compatibility with existing search infrastructure

## UI/UX Considerations
- Two centered icons in navbar: Translate and Search
- Clean, balanced interface design
- Responsive components that work across different devices
- Accessible design with proper ARIA labels and tooltips

## File Structure
- Components organized in `src/components/Translation` and `src/components/Search`
- Services in `src/services/` for business logic
- Utilities in `src/utils/` for transliteration and storage
- API contracts defined in `specs/1-translate-search/contracts/`

## API Endpoints
- `POST /api/translate` - Translation service
- `GET/POST /api/translation-history` - Translation history management
- `GET /api/search` - Enhanced search functionality
- `GET/POST /api/search/history` - Search history management

## Development Workflow
- Follow Docusaurus component patterns for integration
- Use React hooks for state management
- Implement proper error handling and loading states
- Ensure performance optimization for translation and search operations

## Integration Points
- Navbar integration for centered icons
- Book module content integration for translation
- Existing search infrastructure enhancement
- RAG chatbot potential integration points

## Quality Standards
- Translation accuracy for English to Roman Urdu conversion
- Search relevance and performance (sub 2-second response)
- UI balance and clean design
- Browser compatibility across major platforms