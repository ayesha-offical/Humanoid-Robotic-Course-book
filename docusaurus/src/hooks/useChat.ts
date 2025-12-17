import { useState, useCallback } from 'react';

export interface Message {
  role: 'user' | 'assistant';
  content: string;
}

export interface ChatResponse {
  answer: string;
  sources: Array<{ title: string; url: string }>;
  confidence: number;
  session_id: string;
  latency_ms: number;
}

interface ChatState {
  messages: Message[];
  isLoading: boolean;
  error: string | null;
  sessionId: string | null;
}

const API_URL = typeof window !== 'undefined'
  ? (window as any).__API_URL || 'http://localhost:8000'
  : 'http://localhost:8000';

export const useChat = () => {
  const [state, setState] = useState<ChatState>({
    messages: [],
    isLoading: false,
    error: null,
    sessionId: null,
  });

  const chat = useCallback(
    async (
      query: string,
      sessionId?: string | null,
      personalize: boolean = false,
      selectedText?: string | null
    ): Promise<ChatResponse | null> => {
      setState((prev) => ({ ...prev, isLoading: true, error: null }));
      try {
        const response = await fetch(`${API_URL}/api/ask`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            message: query,
            selected_text: selectedText || null,
            user_id: 'guest_user',
            session_id: sessionId || undefined,
            personalize,
          }),
        });

        if (!response.ok) {
          throw new Error('Chat request failed');
        }

        const data: ChatResponse = await response.json();

        setState((prev) => ({
          ...prev,
          messages: [
            ...prev.messages,
            { role: 'user', content: query },
            { role: 'assistant', content: data.answer },
          ],
          sessionId: data.session_id,
          isLoading: false,
        }));

        return data;
      } catch (error) {
        const errorMessage =
          error instanceof Error ? error.message : 'Chat error';
        setState((prev) => ({
          ...prev,
          isLoading: false,
          error: errorMessage,
        }));
        return null;
      }
    },
    []
  );

  const streamChat = useCallback(
    async (query: string, sessionId?: string | null, selectedText?: string | null) => {
      setState((prev) => ({ ...prev, isLoading: true, error: null }));
      try {
        const wsUrl = new URL(API_URL.replace('http', 'ws') + '/ws/chat');

        const ws = new WebSocket(wsUrl.toString());

        return new Promise<void>((resolve, reject) => {
          let fullResponse = '';
          let messageTimeout: NodeJS.Timeout;

          ws.onopen = () => {
            // Clear any existing timeout
            if (messageTimeout) clearTimeout(messageTimeout);

            ws.send(
              JSON.stringify({
                message: query,
                selected_text: selectedText || null,
                user_id: 'guest_user',
                session_id: sessionId || undefined,
              })
            );

            // Set timeout for message processing
            messageTimeout = setTimeout(() => {
              if (ws.readyState === WebSocket.OPEN) {
                ws.close();
              }
              reject(new Error('WebSocket message timeout'));
            }, 60000); // 60 second timeout
          };

          ws.onmessage = (event) => {
            // Reset timeout on message
            if (messageTimeout) clearTimeout(messageTimeout);
            messageTimeout = setTimeout(() => {
              if (ws.readyState === WebSocket.OPEN) {
                ws.close();
              }
              reject(new Error('WebSocket message timeout'));
            }, 60000);

            const chunk = JSON.parse(event.data);
            if (chunk.token) {
              fullResponse += chunk.token;
              setState((prev) => {
                const messages = [...prev.messages];
                if (
                  messages.length === 0 ||
                  messages[messages.length - 1].role === 'user'
                ) {
                  messages.push({
                    role: 'assistant',
                    content: chunk.token,
                  });
                } else {
                  messages[messages.length - 1].content += chunk.token;
                }
                return { ...prev, messages };
              });
            }
            if (chunk.done) {
              if (messageTimeout) clearTimeout(messageTimeout);
              setState((prev) => ({
                ...prev,
                sessionId: chunk.session_id,
                isLoading: false,
              }));
              resolve();
              ws.close();
            }
          };

          ws.onerror = (error) => {
            if (messageTimeout) clearTimeout(messageTimeout);
            const errorMessage = 'WebSocket error';
            setState((prev) => ({
              ...prev,
              isLoading: false,
              error: errorMessage,
            }));
            reject(error);
          };

          ws.onclose = () => {
            if (messageTimeout) clearTimeout(messageTimeout);
          };
        });
      } catch (error) {
        const errorMessage =
          error instanceof Error ? error.message : 'Stream error';
        setState((prev) => ({
          ...prev,
          isLoading: false,
          error: errorMessage,
        }));
        throw error;
      }
    },
    []
  );

  const clearChat = useCallback(() => {
    setState({
      messages: [],
      isLoading: false,
      error: null,
      sessionId: null,
    });
  }, []);

  return {
    ...state,
    chat,
    streamChat,
    clearChat,
  };
};
