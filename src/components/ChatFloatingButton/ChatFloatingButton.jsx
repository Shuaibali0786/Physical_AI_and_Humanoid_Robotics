import React, { useState } from 'react';
import './ChatFloatingButton.css';

const ChatFloatingButton = ({ onClick }) => {
  return (
    <button
      className="chat-fab"
      onClick={onClick}
      aria-label="Open chat"
      title="Chat with Qwen RAG Assistant"
    >
      <div className="chat-icon">
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
          <path d="M12 2C6.48 2 2 6.48 2 12C2 13.54 2.36 15.01 3.02 16.35L2 22L7.65 20.98C8.99 21.64 10.46 22 12 22C17.52 22 22 17.52 22 12C22 6.48 17.52 2 12 2ZM8 16L10 14L8 12L10 10L12 12L14 10L16 12L14 14L16 16L14 18L12 16L10 18L8 16Z" fill="currentColor"/>
        </svg>
      </div>
    </button>
  );
};

export default ChatFloatingButton;