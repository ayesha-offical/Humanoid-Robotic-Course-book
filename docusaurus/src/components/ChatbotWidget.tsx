import React, { useState, useRef, useEffect } from 'react';
import { useChat } from '../hooks/useChat';

interface ChatbotWidgetProps {
  token: string | null;
  personalized?: boolean;
}

export const ChatbotWidget: React.FC<ChatbotWidgetProps> = ({
  token,
  personalized = false,
}) => {
  const { messages, isLoading, error, chat, clearChat } = useChat(token);
  const [input, setInput] = useState('');
  const [isOpen, setIsOpen] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const handleSend = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!input.trim() || !token) return;

    const query = input.trim();
    setInput('');
    await chat(query, undefined, personalized);
  };

  const handleClear = () => {
    clearChat();
    setInput('');
  };

  if (!isOpen) {
    return (
      <button
        onClick={() => setIsOpen(true)}
        className="fixed bottom-6 right-6 bg-blue-600 text-white rounded-full w-14 h-14 flex items-center justify-center shadow-lg hover:bg-blue-700 z-40"
        aria-label="Open chatbot"
      >
        💬
      </button>
    );
  }

  return (
    <div className="fixed bottom-6 right-6 w-96 max-w-[90vw] bg-white rounded-lg shadow-2xl flex flex-col z-50 h-[500px]">
      {/* Header */}
      <div className="bg-blue-600 text-white p-4 rounded-t-lg flex items-center justify-between">
        <h3 className="font-semibold">Physical AI Assistant</h3>
        <div className="flex gap-2">
          {messages.length > 0 && (
            <button
              onClick={handleClear}
              className="text-sm hover:bg-blue-700 px-2 py-1 rounded"
              title="Clear chat"
            >
              🗑️
            </button>
          )}
          <button
            onClick={() => {
              setIsOpen(false);
              setInput('');
            }}
            className="text-sm hover:bg-blue-700 px-2 py-1 rounded"
          >
            ✕
          </button>
        </div>
      </div>

      {/* Messages */}
      <div className="flex-1 overflow-y-auto p-4 space-y-4">
        {messages.length === 0 && (
          <div className="text-center text-gray-500 mt-8">
            <p className="text-sm">
              Welcome! Ask me anything about the Physical AI curriculum.
            </p>
          </div>
        )}

        {messages.map((msg, idx) => (
          <div
            key={idx}
            className={`flex ${
              msg.role === 'user' ? 'justify-end' : 'justify-start'
            }`}
          >
            <div
              className={`max-w-xs px-3 py-2 rounded-lg ${
                msg.role === 'user'
                  ? 'bg-blue-600 text-white'
                  : 'bg-gray-200 text-gray-900'
              }`}
            >
              <p className="text-sm">{msg.content}</p>
            </div>
          </div>
        ))}

        {isLoading && (
          <div className="flex justify-start">
            <div className="bg-gray-200 text-gray-900 px-3 py-2 rounded-lg">
              <div className="flex gap-1">
                <span className="w-2 h-2 bg-gray-600 rounded-full animate-bounce"></span>
                <span className="w-2 h-2 bg-gray-600 rounded-full animate-bounce delay-100"></span>
                <span className="w-2 h-2 bg-gray-600 rounded-full animate-bounce delay-200"></span>
              </div>
            </div>
          </div>
        )}

        {error && (
          <div className="text-center text-red-500 text-sm">
            Error: {error}
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      {!token && (
        <div className="p-4 bg-yellow-50 border-t border-yellow-200 text-sm text-yellow-800">
          Please sign in to use the chatbot
        </div>
      )}

      {/* Input */}
      <form
        onSubmit={handleSend}
        className="border-t p-4 flex gap-2"
      >
        <input
          type="text"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          placeholder={token ? 'Ask a question...' : 'Sign in to chat'}
          disabled={!token || isLoading}
          className="flex-1 px-3 py-2 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500 disabled:bg-gray-100"
        />
        <button
          type="submit"
          disabled={!token || isLoading || !input.trim()}
          className="bg-blue-600 text-white px-4 py-2 rounded-lg hover:bg-blue-700 disabled:bg-gray-400 disabled:cursor-not-allowed"
        >
          Send
        </button>
      </form>
    </div>
  );
};
