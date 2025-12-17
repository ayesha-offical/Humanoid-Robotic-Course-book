import { useState, useCallback } from 'react';

interface PersonalizationProfile {
  user_id: string;
  hardware_background: string;
  software_background: string;
  personalization_enabled: boolean;
  language: string;
  theme: string;
}

interface PersonalizationState {
  profile: PersonalizationProfile | null;
  level: 'beginner' | 'intermediate' | 'advanced' | null;
  isLoading: boolean;
  error: string | null;
}

const API_URL = typeof window !== 'undefined'
  ? (window as any).__API_URL || 'http://localhost:8000'
  : 'http://localhost:8000';

export const usePersonalization = () => {
  const [state, setState] = useState<PersonalizationState>({
    profile: null,
    level: null,
    isLoading: false,
    error: null,
  });

  const getPersonalizationLevel = useCallback(async () => {
    // Load personalization profile from localStorage if available
    const savedProfile = localStorage.getItem('userBackground');
    if (savedProfile) {
      try {
        const parsed = JSON.parse(savedProfile);
        const profile: PersonalizationProfile = {
          user_id: 'guest',
          hardware_background: parsed.hardware || 'beginner',
          software_background: parsed.software || 'none',
          personalization_enabled: true,
          language: 'en',
          theme: 'light',
        };

        // Determine personalization level based on backgrounds
        let level: 'beginner' | 'intermediate' | 'advanced' = 'beginner';
        if (
          profile.hardware_background.includes('advanced') ||
          profile.software_background.includes('advanced')
        ) {
          level = 'advanced';
        } else if (
          profile.hardware_background.includes('intermediate') ||
          profile.software_background.includes('intermediate')
        ) {
          level = 'intermediate';
        }

        setState((prev) => ({
          ...prev,
          profile,
          level,
          isLoading: false,
        }));

        return { profile, level };
      } catch (error) {
        console.error('Failed to parse saved profile:', error);
      }
    }

    setState((prev) => ({ ...prev, isLoading: true, error: null }));
    try {
      const response = await fetch(`${API_URL}/api/personalize/profile`, {
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (!response.ok) {
        throw new Error('Failed to fetch personalization profile');
      }

      const profile: PersonalizationProfile = await response.json();

      // Determine personalization level based on backgrounds
      let level: 'beginner' | 'intermediate' | 'advanced' = 'beginner';
      if (
        profile.hardware_background.includes('advanced') ||
        profile.software_background.includes('advanced')
      ) {
        level = 'advanced';
      } else if (
        profile.hardware_background.includes('intermediate') ||
        profile.software_background.includes('intermediate')
      ) {
        level = 'intermediate';
      }

      setState((prev) => ({
        ...prev,
        profile,
        level,
        isLoading: false,
      }));

      return { profile, level };
    } catch (error) {
      const errorMessage =
        error instanceof Error ? error.message : 'Error fetching profile';
      setState((prev) => ({
        ...prev,
        isLoading: false,
        error: errorMessage,
      }));
      return null;
    }
  }, []);

  const setPersonalizationEnabled = useCallback(
    async (enabled: boolean) => {
      setState((prev) => ({ ...prev, isLoading: true, error: null }));
      try {
        const response = await fetch(`${API_URL}/api/personalize/profile`, {
          method: 'PUT',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            personalization_enabled: enabled,
          }),
        });

        if (!response.ok) {
          throw new Error('Failed to update personalization settings');
        }

        const updatedProfile: PersonalizationProfile =
          await response.json();
        setState((prev) => ({
          ...prev,
          profile: updatedProfile,
          isLoading: false,
        }));

        return true;
      } catch (error) {
        const errorMessage =
          error instanceof Error ? error.message : 'Update error';
        setState((prev) => ({
          ...prev,
          isLoading: false,
          error: errorMessage,
        }));
        return false;
      }
    },
    []
  );

  return {
    ...state,
    getPersonalizationLevel,
    setPersonalizationEnabled,
  };
};
