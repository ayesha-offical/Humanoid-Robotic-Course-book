import React, { useState, useEffect } from 'react';
import { usePersonalization } from '../hooks/usePersonalization';

interface PersonalizeButtonProps {
  token: string | null;
  onPersonalizeChange?: (personalized: boolean) => void;
}

export const PersonalizeButton: React.FC<PersonalizeButtonProps> = ({
  token,
  onPersonalizeChange,
}) => {
  const { profile, level, getPersonalizationLevel, setPersonalizationEnabled } =
    usePersonalization(token);
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [isLoading, setIsLoading] = useState(false);

  useEffect(() => {
    if (token) {
      getPersonalizationLevel();
    }
  }, [token, getPersonalizationLevel]);

  const handleToggle = async () => {
    setIsLoading(true);
    const newState = !isPersonalized;
    const success = await setPersonalizationEnabled(newState);
    if (success) {
      setIsPersonalized(newState);
      onPersonalizeChange?.(newState);
    }
    setIsLoading(false);
  };

  if (!token) {
    return null;
  }

  return (
    <button
      onClick={handleToggle}
      disabled={isLoading}
      className={`inline-flex items-center gap-2 px-4 py-2 rounded-lg font-medium transition-colors ${
        isPersonalized
          ? 'bg-green-600 text-white hover:bg-green-700'
          : 'bg-gray-200 text-gray-900 hover:bg-gray-300'
      } disabled:opacity-50 disabled:cursor-not-allowed`}
      title={`Personalization: ${level || 'unknown'}`}
    >
      <span>🎯</span>
      <span>{isPersonalized ? 'ON' : 'OFF'}</span>
      {level && <span className="text-sm ml-1">({level})</span>}
    </button>
  );
};
