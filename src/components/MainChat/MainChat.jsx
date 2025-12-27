import React, { useState } from 'react';
import ChatFloatingButton from '../ChatFloatingButton/ChatFloatingButton';
import ChatSidebar from '../ChatSidebar/ChatSidebar';
import './MainChat.css';

const MainChat = () => {
  const [isChatOpen, setIsChatOpen] = useState(false);

  const toggleChat = () => {
    setIsChatOpen(!isChatOpen);
  };

  const closeChat = () => {
    setIsChatOpen(false);
  };

  return (
    <>
      <ChatFloatingButton onClick={toggleChat} />
      {isChatOpen && (
        <ChatSidebar
          isOpen={isChatOpen}
          onClose={closeChat}
          onSendMessage={(message) => console.log('Sending message:', message)}
        />
      )}
    </>
  );
};

export default MainChat;