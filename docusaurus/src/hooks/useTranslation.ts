import { useState, useCallback } from 'react';

interface SupportedLanguage {
  code: string;
  name: string;
}

interface TranslationResponse {
  chapter_id: string;
  language: string;
  translated_title: string;
  translated_summary: string;
  cached: boolean;
}

interface TranslationState {
  translations: Record<string, TranslationResponse>;
  supportedLanguages: SupportedLanguage[];
  isLoading: boolean;
  error: string | null;
}

const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

export const useTranslation = () => {
  const [state, setState] = useState<TranslationState>({
    translations: {},
    supportedLanguages: [],
    isLoading: false,
    error: null,
  });

  const getSupportedLanguages = useCallback(async () => {
    setState((prev) => ({ ...prev, isLoading: true, error: null }));
    try {
      const response = await fetch(
        `${API_URL}/api/translate/supported-languages`
      );

      if (!response.ok) {
        throw new Error('Failed to fetch supported languages');
      }

      const languages: SupportedLanguage[] = await response.json();
      setState((prev) => ({
        ...prev,
        supportedLanguages: languages,
        isLoading: false,
      }));

      return languages;
    } catch (error) {
      const errorMessage =
        error instanceof Error ? error.message : 'Error fetching languages';
      setState((prev) => ({
        ...prev,
        isLoading: false,
        error: errorMessage,
      }));
      return [];
    }
  }, []);

  const getTranslation = useCallback(
    async (
      content_id: string,
      language: string = 'ur'
    ): Promise<TranslationResponse | null> => {
      setState((prev) => ({ ...prev, isLoading: true, error: null }));
      try {
        const response = await fetch(
          `${API_URL}/api/translate/chapter?chapter_id=${content_id}&language=${language}`
        );

        if (!response.ok) {
          throw new Error('Failed to fetch translation');
        }

        const translation: TranslationResponse = await response.json();
        setState((prev) => ({
          ...prev,
          translations: {
            ...prev.translations,
            [`${content_id}-${language}`]: translation,
          },
          isLoading: false,
        }));

        return translation;
      } catch (error) {
        const errorMessage =
          error instanceof Error ? error.message : 'Error fetching translation';
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

  const clearCache = useCallback(() => {
    setState((prev) => ({
      ...prev,
      translations: {},
    }));
  }, []);

  return {
    ...state,
    getTranslation,
    getSupportedLanguages,
    clearCache,
  };
};
