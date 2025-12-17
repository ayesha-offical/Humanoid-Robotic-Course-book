import React, { useState } from 'react';

interface BackgroundExpertiseFormProps {
  onSuccess?: () => void;
  onCancel?: () => void;
  showSkipButton?: boolean;
}

export const BackgroundExpertiseForm: React.FC<BackgroundExpertiseFormProps> = ({
  onSuccess,
  onCancel,
  showSkipButton = true,
}) => {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [formData, setFormData] = useState({
    hardware_background: 'beginner',
    software_background: 'none',
  });

  const handleInputChange = (
    e: React.ChangeEvent<HTMLSelectElement>
  ) => {
    const { name, value } = e.target;
    setFormData((prev) => ({
      ...prev,
      [name]: value,
    }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsLoading(true);
    setError(null);
    try {
      // Store preference locally
      localStorage.setItem('userBackground', JSON.stringify({
        hardware: formData.hardware_background,
        software: formData.software_background
      }));
      onSuccess?.();
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to save preferences');
      console.error('Failed to save background:', err);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="bg-white rounded-lg shadow-lg p-6 max-w-md w-full mx-4">
      <h2 className="text-2xl font-bold text-gray-900 mb-6">
        Tell us about your background
      </h2>

      {error && (
        <div className="mb-4 p-3 bg-red-100 text-red-700 rounded">
          {error}
        </div>
      )}

      <form onSubmit={handleSubmit} className="space-y-4">
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-2">
            Hardware & Robotics Background
          </label>
          <select
            name="hardware_background"
            value={formData.hardware_background}
            onChange={handleInputChange}
            className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
          >
            <option value="beginner">Beginner - I'm new to robotics</option>
            <option value="intermediate">
              Intermediate - I have some robotics experience
            </option>
            <option value="advanced">Advanced - I'm experienced with robotics</option>
          </select>
          <p className="mt-1 text-sm text-gray-500">
            This helps us tailor hardware examples to your level
          </p>
        </div>

        <div>
          <label className="block text-sm font-medium text-gray-700 mb-2">
            Software & AI Background
          </label>
          <select
            name="software_background"
            value={formData.software_background}
            onChange={handleInputChange}
            className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
          >
            <option value="none">None - New to programming</option>
            <option value="some_python">
              Some Python - I know basic Python
            </option>
            <option value="advanced_ml">Advanced - I know AI/ML & Python</option>
          </select>
          <p className="mt-1 text-sm text-gray-500">
            This helps us adapt code examples to your skill level
          </p>
        </div>

        <div className="flex gap-3 pt-4">
          <button
            type="submit"
            disabled={isLoading}
            className="flex-1 bg-blue-600 text-white py-2 rounded-md hover:bg-blue-700 disabled:bg-gray-400 font-medium transition-colors"
          >
            {isLoading ? 'Saving...' : 'Save & Continue'}
          </button>
          {showSkipButton && (
            <button
              type="button"
              onClick={onCancel}
              className="flex-1 bg-gray-200 text-gray-700 py-2 rounded-md hover:bg-gray-300 font-medium transition-colors"
            >
              Skip for now
            </button>
          )}
        </div>
      </form>
    </div>
  );
};
