import React, { useState, useRef, useEffect } from 'react';
import { ChatMessage, ChatSession, PageContext } from '../types/rag-chat.types';
import apiService from '../services/api.service';
import { generateId, getCurrentPageContext, sanitizeInput } from '../utils/rag-utils';
import './rag-chat.css';

interface RagChatProps {
  pageContext?: PageContext;
  apiUrl?: string;
}

const RagChat: React.FC<RagChatProps> = ({ pageContext, apiUrl }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [session, setSession] = useState<ChatSession>({
    id: generateId(),
    messages: [],
    isLoading: false,
  });

  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Initialize with page context if not provided
  const resolvedPageContext = pageContext || (typeof window !== 'undefined'
    ? getCurrentPageContext()
    : { url: '', title: 'Unknown Page' });

  useEffect(() => {
    if (apiUrl) {
      // Update API service with custom URL if provided
      // This would require modifying the apiService to be configurable
      // For now, we'll use the default configured URL
    }
  }, [apiUrl]);

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!inputValue.trim() || isLoading) return;

    // Sanitize the input
    const sanitizedInput = sanitizeInput(inputValue);
    if (!sanitizedInput) {
      setError('Invalid input provided');
      return;
    }

    // Create user message
    const userMessage: ChatMessage = {
      id: generateId(),
      text: sanitizedInput,
      sender: 'user',
      timestamp: Date.now(),
    };

    // Add user message to state
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      // Query the RAG agent
      const response = await apiService.queryAgent(
        sanitizedInput,
        resolvedPageContext
      );

      // Create agent message
      const agentMessage: ChatMessage = {
        id: generateId(),
        text: response.answer,
        sender: 'agent',
        timestamp: Date.now(),
        sources: response.sources,
      };

      // Add agent message to state
      setMessages(prev => [...prev, agentMessage]);
    } catch (err) {
      console.error('Error querying RAG agent:', err);

      // Create error message
      const errorMessage: ChatMessage = {
        id: generateId(),
        text: 'Sorry, I encountered an error processing your request. Please try again.',
        sender: 'agent',
        timestamp: Date.now(),
      };

      setMessages(prev => [...prev, errorMessage]);
      setError(err instanceof Error ? err.message : 'Unknown error occurred');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={`rag-chat-container ${isOpen ? 'open' : 'closed'}`}>
      {!isOpen ? (
        <button
          className="chat-toggle"
          onClick={() => setIsOpen(true)}
          aria-label="Open chat"
        >
          💬 Ask AI
        </button>
      ) : (
        <div className="chat-window">
          <div className="chat-header">
            <h3>Book Assistant</h3>
            <button
              className="chat-toggle"
              onClick={() => setIsOpen(false)}
              aria-label="Close chat"
            >
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
                  className={`message ${message.sender} ${message.sender === 'agent' && message.sources ? 'has-sources' : ''}`}
                >
                  <div className="message-text">{message.text}</div>
                  {message.sources && message.sources.length > 0 && (
                    <div className="message-sources">
                      Sources: {message.sources.map((source, idx) => (
                        <React.Fragment key={idx}>
                          {idx > 0 && ', '}
                          <a href={source} target="_blank" rel="noopener noreferrer">
                            {new URL(source).hostname}
                          </a>
                        </React.Fragment>
                      ))}
                    </div>
                  )}
                </div>
              ))
            )}
            {isLoading && (
              <div className="message agent">
                <div className="loading-indicator">
                  <span>Thinking</span>
                  <span className="loading-dots"></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {error && (
            <div className="error-message">
              Error: {error}
            </div>
          )}

          <form onSubmit={handleSubmit} className="chat-input-form">
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask about this page..."
              disabled={isLoading}
              aria-label="Type your question"
            />
            <button
              type="submit"
              disabled={isLoading || !inputValue.trim()}
              aria-label="Send question"
            >
              Send
            </button>
          </form>

          <div className="chat-footer">
            Powered by RAG Agent
          </div>
        </div>
      )}
    </div>
  );
};

export default RagChat;