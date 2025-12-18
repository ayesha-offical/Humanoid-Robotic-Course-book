import React, { useState, useRef, useEffect } from 'react';
import './styles.css';

interface Message {
  id: string;
  text: string;
  sender: 'user' | 'bot';
  citations?: string[];
  timestamp: number;
}

interface ChatResponse {
  answer: string;
  sources?: string[];
  citations?: string[];
  confidence?: number;
  latency_ms?: number;
}

const ChatWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([
    {
      id: '1',
      text: 'Hello! I\'m your AI assistant for the Physical AI & Humanoid Robotics course. Ask me anything about ROS 2, Gazebo, Isaac Sim, or humanoid robotics Course book!',
      sender: 'bot',
      timestamp: Date.now(),
    },
  ]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  // Backend API URL - configurable via environment
  const API_URL = typeof window !== 'undefined' && (window as any).__CHAT_API_URL__
    ? (window as any).__CHAT_API_URL__
    : 'http://localhost:8000/api/chat';

  // Auto-scroll to latest message
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Auto-focus input when window opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      setTimeout(() => inputRef.current?.focus(), 100);
    }
  }, [isOpen]);

  // Handle textarea auto-resize
  useEffect(() => {
    if (inputRef.current) {
      inputRef.current.style.height = 'auto';
      inputRef.current.style.height = `${Math.min(inputRef.current.scrollHeight, 100)}px`;
    }
  }, [input]);

  const sendMessage = async () => {
    if (!input.trim()) return;

    // Add user message
    const userMessage: Message = {
      id: `msg-${Date.now()}`,
      text: input,
      sender: 'user',
      timestamp: Date.now(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);
    setError(null);

    try {
      // Call backend API
      const response = await fetch(API_URL, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: userMessage.text,
          session_id: undefined, // Optional for stateless requests
        }),
      });

      if (!response.ok) {
        throw new Error(`API Error: ${response.status} ${response.statusText}`);
      }

      const data: ChatResponse = await response.json();

      // Add bot response
      const botMessage: Message = {
        id: `msg-${Date.now()}`,
        text: data.answer || 'Sorry, I could not generate a response.',
        sender: 'bot',
        citations: data.sources || data.citations || [],
        timestamp: Date.now(),
      };

      setMessages((prev) => [...prev, botMessage]);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Unknown error occurred';

      // Add error message to chat
      const errorMsg: Message = {
        id: `msg-${Date.now()}`,
        text: `Error: ${errorMessage}. Wait and refresh it will fix it.`,
        sender: 'bot',
        timestamp: Date.now(),
      };

      setMessages((prev) => [...prev, errorMsg]);
      setError(errorMessage);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <>
      {/* Floating Action Button */}
      <button
        className="chat-widget-fab"
        onClick={() => setIsOpen(!isOpen)}
        title={isOpen ? 'Close chat' : 'Open chat'}
        aria-label="Toggle chat widget"
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className="chat-widget-window">
          {/* Header */}
          <div className="chat-widget-header">
            <h3>AI Course Assistant</h3>
            <button
              className="chat-widget-close"
              onClick={() => setIsOpen(false)}
              aria-label="Close chat"
            >
              ✕
            </button>
          </div>

          {/* Messages */}
          <div className="chat-widget-messages">
            {messages.map((msg) => (
              <div key={msg.id} className={`chat-widget-message ${msg.sender}`}>
                <div className="chat-widget-message-bubble">
                  {msg.text}
                  {msg.citations && msg.citations.length > 0 && (
                    <div className="chat-widget-citations">
                      <div className="chat-widget-citations-title">Sources:</div>
                      {msg.citations.map((citation, idx) => (
                        <div key={idx} className="chat-widget-citation-item">
                          {citation.startsWith('http') ? (
                            <a href={citation} target="_blank" rel="noopener noreferrer">
                              Source {idx + 1}
                            </a>
                          ) : (
                            <span>{citation}</span>
                          )}
                        </div>
                      ))}
                    </div>
                  )}
                </div>
              </div>
            ))}

            {/* Loading Indicator */}
            {isLoading && (
              <div className="chat-widget-message bot">
                <div className="chat-widget-message-bubble chat-widget-typing">
                  <span className="chat-widget-typing-dot"></span>
                  <span className="chat-widget-typing-dot"></span>
                  <span className="chat-widget-typing-dot"></span>
                </div>
              </div>
            )}

            {/* Error Display */}
            {error && (
              <div className="chat-widget-error">
                {error}
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input Area */}
          <div className="chat-widget-input-area">
            <textarea
              ref={inputRef}
              className="chat-widget-input"
              placeholder="Ask me anything... (Shift+Enter for new line)"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              disabled={isLoading}
              rows={1}
            />
            <button
              className="chat-widget-send-btn"
              onClick={sendMessage}
              disabled={isLoading || !input.trim()}
              title="Send message (Enter)"
            >
              Send
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default ChatWidget;
