import React, { useState, useEffect, useRef } from 'react';
import axios from 'axios';
import './chat.css';

// --- SVG Icons for the new futuristic UI ---

// 1. Robot Avatar with updated green/teal gradient
const RobotAvatarIcon = () => (
  <svg width="40" height="40" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M12 2a2 2 0 0 0-2 2v2H8a2 2 0 0 0-2 2v2H4a2 2 0 0 0-2 2v4a2 2 0 0 0 2 2h2v2a2 2 0 0 0 2 2h4a2 2 0 0 0 2-2v-2h2a2 2 0 0 0 2-2v-4a2 2 0 0 0-2-2h-2V8a2 2 0 0 0-2-2h-4V4a2 2 0 0 0-2-2zM9 12a1 1 0 1 1-2 0 1 1 0 0 1 2 0zm6 0a1 1 0 1 1-2 0 1 1 0 0 1 2 0z" fill="url(#robot-gradient)"/>
    <defs>
      <linearGradient id="robot-gradient" x1="0" y1="0" x2="1" y2="1">
        <stop offset="0%" stopColor="#24c19e"/>
        <stop offset="100%" stopColor="#21af90"/>
      </linearGradient>
    </defs>
  </svg>
);

// 2. Send Arrow Icon for the input button
const SendIcon = () => (
  <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M3 12h18m-9-9l9 9-9 9" stroke="white" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
  </svg>
);

// 3. Emoji Icon placeholder
const EmojiIcon = () => (
    <svg width="22" height="22" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M12 22C17.5228 22 22 17.5228 22 12C22 6.47715 17.5228 2 12 2C6.47715 2 2 6.47715 2 12C2 17.5228 6.47715 22 12 22Z" stroke="#888" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
        <path d="M8 14C8 14 9.5 16 12 16C14.5 16 16 14 16 14" stroke="#888" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
        <path d="M9 9.01L9.01 8.99889" stroke="#888" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
        <path d="M15 9.01L15.01 8.99889" stroke="#888" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
    </svg>
);

// 4. Attachment Icon placeholder
const AttachmentIcon = () => (
    <svg width="22" height="22" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M14.5 11.5L9.5 16.5L4.5 11.5" stroke="#888" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
        <path d="M9.5 16.5V2.5" stroke="#888" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
        <path d="M20.5 11.5C20.5 16.5 15.5 21.5 9.5 21.5C3.5 21.5 -1.5 16.5 0.5 11.5" stroke="#888" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
    </svg>
);


export default function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState("");
  const [loading, setLoading] = useState(false);
  const chatBodyRef = useRef(null);

  useEffect(() => {
    if (chatBodyRef.current) {
      chatBodyRef.current.scrollTop = chatBodyRef.current.scrollHeight;
    }
  }, [messages, loading]);
  
  useEffect(() => {
    setMessages([
      { sender: 'bot', text: 'Greetings. I am the Physical AI assistant. How may I be of service?' }
    ]);
  }, []);

  const sendMessage = async () => {
    if (!input.trim()) return;

    setMessages(prev => [...prev, { sender: 'user', text: input }]);
    const msg = input;
    setInput("");
    setLoading(true);

    try {
      const res = await axios.post(
        "https://shumii029-physical-ai-chatbot.hf.space/api/predict",
        { data: [msg] }
      );
      const botReply = res.data?.data?.[0] || "I seem to be having trouble connecting. Please try again.";
      setMessages(prev => [...prev, { sender: 'bot', text: botReply }]);
    } catch (err) {
      console.error("Chat API error:", err);
      setMessages(prev => [
        ...prev,
        { sender: 'bot', text: "Connection error. Please check the console for details." }
      ]);
    } finally {
      setLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === "Enter" && !loading) sendMessage();
  };

  return (
    <div className="chat-widget-container">
      <div className={`chat-box ${isOpen ? 'open' : ''}`}>
        <div className="chat-header">
          <div className="header-content">
            <div className="avatar-container">
              <RobotAvatarIcon />
              <div className="status-dot"></div>
            </div>
            <div className="header-title">AI Assistant</div>
          </div>
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
          <div className="input-wrapper">
            <input
              type="text"
              value={input}
              onChange={e => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Type a message..."
              disabled={loading}
            />
            <div className="input-icons">
              <AttachmentIcon />
              <EmojiIcon />
            </div>
          </div>
          <button onClick={sendMessage} disabled={loading} className="send-button">
            <SendIcon />
          </button>
        </div>
      </div>

      <button className="chat-toggle-bubble" onClick={() => setIsOpen(!isOpen)}>
        <RobotAvatarIcon />
      </button>
    </div>
  );
}
