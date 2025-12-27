import React, { useState, useRef, useEffect } from 'react';
import './ChatSidebar.css';

const ChatSidebar = ({ isOpen, onClose, onSendMessage }) => {
  const [message, setMessage] = useState('');
  const [messages, setMessages] = useState([
    { id: 1, text: 'Hello! I\'m your Qwen RAG Assistant. How can I help you with the book content today?', sender: 'assistant' }
  ]);
  const [isTyping, setIsTyping] = useState(false);
  const [inputActive, setInputActive] = useState(false);
  const [targetLanguage, setTargetLanguage] = useState('english'); // Add language state
  const messagesEndRef = useRef(null);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (message.trim()) {
      const userMessage = {
        id: Date.now(),
        text: message,
        sender: 'user'
      };

      setMessages(prev => [...prev, userMessage]);
      setMessage('');
      setIsTyping(true);

      try {
        // Send message to Qwen RAG backend
        const response = await fetch('http://localhost:8000/api/qwen-chat', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            query: message,
            context: window.location.pathname, // Pass current page context
            target_language: targetLanguage // Add target language for translation
          }),
        });

        setIsTyping(false);

        if (response.ok) {
          const data = await response.json();
          const assistantMessage = {
            id: Date.now() + 1,
            text: data.response || 'Sorry, I could not generate a response.',
            sender: 'assistant'
          };
          setMessages(prev => [...prev, assistantMessage]);
        } else {
          // Try to get error details from response
          let errorText = 'Sorry, there was an error processing your request.';
          try {
            const errorData = await response.json();
            if (errorData.detail) {
              errorText = `Error: ${errorData.detail}`;
            }
          } catch (parseError) {
            // If we can't parse the error, use the default message
          }

          const errorMessage = {
            id: Date.now() + 1,
            text: errorText,
            sender: 'assistant'
          };
          setMessages(prev => [...prev, errorMessage]);
        }
      } catch (error) {
        setIsTyping(false);
        console.error('Chat error:', error);
        const errorMessage = {
          id: Date.now() + 1,
          text: 'Sorry, there was a network error. Please make sure the Qwen API server is running on http://localhost:8000',
          sender: 'assistant'
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    }
  };

  useEffect(() => {
    // Scroll to bottom when messages change
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, isTyping]);

  if (!isOpen) return null;

  return (
    <div className="chat-sidebar-overlay">
      <div className="chat-sidebar">
        {/* Modern Navbar with Glassmorphism Effect */}
        <div className="chat-header">
          <div className="chat-header-content">
            <div className="ai-icon">AI</div>
            <h3 className="chat-title">Qwen RAG Assistant</h3>
          </div>
          <div className="chat-header-controls">
            {/* Language Toggle */}
            <select
              className="language-toggle"
              value={targetLanguage}
              onChange={(e) => setTargetLanguage(e.target.value)}
              title="Select response language"
            >
              <option value="english">English</option>
              <option value="roman_urdu">Roman Urdu</option>
            </select>
            <button className="close-button" onClick={onClose} aria-label="Close chat">
              ×
            </button>
          </div>
        </div>

        <div className="chat-messages">
          {messages.map((msg) => (
            <div
              key={msg.id}
              className={`message ${msg.sender === 'user' ? 'user-message' : 'assistant-message'}`}
            >
              {msg.text}
            </div>
          ))}

          {/* Typing Indicator */}
          {isTyping && (
            <div className="typing-indicator">
              <div className="typing-dot"></div>
              <div className="typing-dot"></div>
              <div className="typing-dot"></div>
            </div>
          )}

          <div ref={messagesEndRef} />
        </div>

        <form
          className="chat-input-form"
          onSubmit={handleSubmit}
          onFocus={() => setInputActive(true)}
          onBlur={() => setInputActive(false)}
        >
          <input
            type="text"
            value={message}
            onChange={(e) => setMessage(e.target.value)}
            placeholder="Ask about the book content..."
            className={`chat-input ${inputActive ? 'input-active' : ''}`}
            autoComplete="off"
            autoCorrect="off"
            autoCapitalize="off"
            spellCheck="false"
          />
          <button
            type="submit"
            className="send-button"
            disabled={!message.trim()}
            aria-label="Send message"
          >
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M22 2L11 13M22 2L15 22L11 13L2 9L22 2Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
            </svg>
          </button>
        </form>
      </div>
    </div>
  );
};

export default ChatSidebar;