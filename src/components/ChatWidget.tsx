import React, { useState, useEffect, useRef } from 'react';
import axios from 'axios';
import './chat.css';

// SVG Icon for the send button
const SendIcon = () => (
  <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M3.47821 2.99994L21.4999 12L3.47821 21L3.5 15.6667L15.4999 12L3.5 8.33331L3.47821 2.99994Z" fill="white"/>
  </svg>
);

// SVG Icon for the chat bubble
const ChatIcon = () => (
  <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M20 2H4C2.9 2 2 2.9 2 4V22L6 18H20C21.1 18 22 17.1 22 16V4C22 2.9 21.1 2 20 2ZM20 16H5.17L4 17.17V4H20V16Z" fill="white"/>
  </svg>
);

export default function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState("");
  const [loading, setLoading] = useState(false);
  const chatBodyRef = useRef(null);

  // Automatically scroll to the bottom of the chat
  useEffect(() => {
    if (chatBodyRef.current) {
      chatBodyRef.current.scrollTop = chatBodyRef.current.scrollHeight;
    }
  }, [messages, loading]);
  
  // Add initial bot message
  useEffect(() => {
    setMessages([
      { sender: 'bot', text: 'Hello! How can I help you with Physical AI today?' }
    ]);
  }, []);

  const sendMessage = async () => {
    if (!input.trim()) return;

    const userMessage = { sender: 'user', text: input };
    const msg = input;

    setMessages(prev => [...prev, userMessage]);
    setInput("");
    setLoading(true);

    try {
      // HuggingFace Space API URL
      const res = await axios.post(
        "https://shumii029-physical-ai-chatbot.hf.space/api/predict",
        {
          data: [msg]
        }
      );

      const botReply = res.data?.data?.[0] || "Sorry, I encountered an issue and can't respond right now.";

      setMessages(prev => [...prev, { sender: 'bot', text: botReply }]);
    } catch (err) {
      console.error("Chat API error:", err);
      const errorMsg = "There was a problem connecting to the server. Please try again later.";
      setMessages(prev => [
        ...prev,
        { sender: 'bot', text: errorMsg }
      ]);
    } finally {
      setLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === "Enter" && !loading) {
      sendMessage();
    }
  };

  return (
    <div className="chat-widget-container">
      <div className={`chat-box ${isOpen ? 'open' : ''}`}>
        <div className="chat-header">
          <div className="chat-header-title">AI Assistant</div>
          <button className="chat-close-button" onClick={() => setIsOpen(false)}>&times;</button>
        </div>

        <div className="chat-body" ref={chatBodyRef}>
          {messages.map((m, i) => (
            <div key={i} className={`chat-bubble ${m.sender}`}>
              {m.text}
            </div>
          ))}
          {loading && (
            <div className="chat-bubble bot typing">
              <span></span><span></span><span></span>
            </div>
          )}
        </div>

        <div className="chat-input-area">
          <input
            type="text"
            value={input}
            onChange={e => setInput(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder="Ask about Physical AI..."
            disabled={loading}
          />
          <button onClick={sendMessage} disabled={loading} className="send-button">
            <SendIcon />
          </button>
        </div>
      </div>

      <button className="chat-toggle-button" onClick={() => setIsOpen(!isOpen)}>
        <span className="chat-toggle-text">Chat with AI</span>
        <ChatIcon />
      </button>
    </div>
  );
}