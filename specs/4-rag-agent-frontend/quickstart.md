# Quickstart: RAG Agent Frontend Integration

## Overview
This guide provides a quick setup for integrating the RAG agent with your Docusaurus frontend.

## Prerequisites
- Docusaurus project already set up
- Backend RAG agent service running (FastAPI server)
- Node.js and npm installed

## Backend Setup
1. Ensure the RAG agent backend is running:
   ```bash
   cd backend
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r requirements.txt
   python -m rag_agent.main
   ```
   The backend should be available at `http://localhost:8000`

## Frontend Integration

### 1. Install Dependencies
```bash
npm install
```

### 2. Create the Chat Component
Create a new React component for the chat interface:

```jsx
// src/components/RagChat.jsx
import React, { useState, useRef } from 'react';
import './RagChat.css';

const RagChat = ({ pageContext }) => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isOpen, setIsOpen] = useState(false);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  React.useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: Date.now()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Add current page context to the request
      const requestBody = {
        query: inputValue,
        pageContext: pageContext || null
      };

      const response = await fetch('http://localhost:8000/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status}`);
      }

      const data = await response.json();

      const agentMessage = {
        id: Date.now() + 1,
        text: data.answer,
        sender: 'agent',
        timestamp: Date.now(),
        sources: data.sources
      };

      setMessages(prev => [...prev, agentMessage]);
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error processing your request. Please try again.',
        sender: 'agent',
        timestamp: Date.now(),
        error: true
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={`rag-chat-container ${isOpen ? 'open' : 'closed'}`}>
      {!isOpen ? (
        <button className="chat-toggle" onClick={() => setIsOpen(true)}>
          💬 Ask AI
        </button>
      ) : (
        <div className="chat-window">
          <div className="chat-header">
            <h3>Book Assistant</h3>
            <button className="chat-toggle" onClick={() => setIsOpen(false)}>
              ×
            </button>
          </div>
          <div className="chat-messages">
            {messages.length === 0 ? (
              <div className="welcome-message">
                Ask me anything about this book!
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`message ${message.sender} ${message.error ? 'error' : ''}`}
                >
                  <div className="message-text">{message.text}</div>
                  {message.sources && message.sources.length > 0 && (
                    <div className="message-sources">
                      Sources: {message.sources.join(', ')}
                    </div>
                  )}
                </div>
              ))
            )}
            {isLoading && (
              <div className="message agent">
                <div className="message-text">
                  <span className="loading-dots">Thinking...</span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
          <form onSubmit={handleSubmit} className="chat-input-form">
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask about this page..."
              disabled={isLoading}
            />
            <button type="submit" disabled={isLoading || !inputValue.trim()}>
              Send
            </button>
          </form>
        </div>
      )}
    </div>
  );
};

export default RagChat;
```

### 3. Add CSS Styling
Create the corresponding CSS file:

```css
/* src/components/RagChat.css */
.rag-chat-container {
  position: fixed;
  bottom: 20px;
  right: 20px;
  z-index: 1000;
}

.rag-chat-container.closed .chat-toggle {
  width: 60px;
  height: 60px;
  border-radius: 50%;
  background: #0077cc;
  color: white;
  border: none;
  font-size: 24px;
  cursor: pointer;
  box-shadow: 0 4px 8px rgba(0,0,0,0.2);
  display: flex;
  align-items: center;
  justify-content: center;
}

.chat-window {
  width: 350px;
  height: 500px;
  border: 1px solid #ddd;
  border-radius: 8px;
  display: flex;
  flex-direction: column;
  box-shadow: 0 4px 12px rgba(0,0,0,0.15);
  background: white;
  overflow: hidden;
}

.chat-header {
  background: #0077cc;
  color: white;
  padding: 12px;
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.chat-header h3 {
  margin: 0;
  font-size: 16px;
}

.chat-toggle {
  background: none;
  border: none;
  color: white;
  font-size: 20px;
  cursor: pointer;
  padding: 0;
  width: 30px;
  height: 30px;
  display: flex;
  align-items: center;
  justify-content: center;
}

.chat-messages {
  flex: 1;
  padding: 16px;
  overflow-y: auto;
  display: flex;
  flex-direction: column;
  gap: 12px;
}

.welcome-message {
  color: #666;
  font-style: italic;
  text-align: center;
  margin-top: 50%;
  transform: translateY(-50%);
}

.message {
  max-width: 80%;
  padding: 8px 12px;
  border-radius: 18px;
  word-wrap: break-word;
}

.message.user {
  align-self: flex-end;
  background: #0077cc;
  color: white;
}

.message.agent {
  align-self: flex-start;
  background: #f0f0f0;
  color: #333;
}

.message.error {
  background: #ffebee;
  color: #c62828;
}

.message-sources {
  font-size: 12px;
  color: #666;
  margin-top: 4px;
}

.loading-dots:after {
  content: '.';
  animation: dots 1.5s infinite;
}

@keyframes dots {
  0%, 20% { content: '.'; }
  40% { content: '..'; }
  60%, 100% { content: '...'; }
}

.chat-input-form {
  display: flex;
  padding: 12px;
  border-top: 1px solid #eee;
  background: white;
}

.chat-input-form input {
  flex: 1;
  padding: 8px 12px;
  border: 1px solid #ddd;
  border-radius: 18px;
  outline: none;
}

.chat-input-form input:focus {
  border-color: #0077cc;
}

.chat-input-form button {
  margin-left: 8px;
  padding: 8px 16px;
  background: #0077cc;
  color: white;
  border: none;
  border-radius: 18px;
  cursor: pointer;
}

.chat-input-form button:disabled {
  background: #ccc;
  cursor: not-allowed;
}
```

### 4. Integrate into Docusaurus Page
To use the chat component on a specific page, import and include it:

```jsx
// In your Docusaurus page
import RagChat from '@site/src/components/RagChat';

function MyPage() {
  const pageContext = {
    url: typeof window !== 'undefined' ? window.location.href : '',
    title: 'Current Page Title',
    contentSnippet: 'Relevant content from this page...'
  };

  return (
    <div>
      <h1>My Page Content</h1>
      {/* Your page content here */}
      <RagChat pageContext={pageContext} />
    </div>
  );
}
```

### 5. Environment Configuration
Add the backend API URL to your Docusaurus configuration:

```js
// docusaurus.config.js
module.exports = {
  // ... other config
  themeConfig: {
    // ... other theme config
    ragAgent: {
      apiUrl: process.env.RAG_AGENT_API_URL || 'http://localhost:8000',
    },
  },
};
```

## Testing
1. Start the backend: `python -m rag_agent.main`
2. Start Docusaurus: `npm run start`
3. Navigate to a page with the chat component
4. Try asking questions about the book content

## Troubleshooting
- If the chat doesn't appear, check that the component is properly imported
- If API calls fail, verify the backend is running and the URL is correct
- Check browser console for any JavaScript errors