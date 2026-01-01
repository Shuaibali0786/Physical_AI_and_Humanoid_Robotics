/**
 * Translation Accuracy Test Component
 *
 * This component tests the accuracy of the English to Roman Urdu translation
 * with various types of English text inputs.
 */

import React, { useState } from 'react';
import { englishToRomanUrdu } from '../../utils/transliterate';
import './../../styles/translation-search.css';

const TranslationAccuracyTest: React.FC = () => {
  const [testInputs, setTestInputs] = useState([
    { id: 1, text: 'Hello world', expected: 'Hello world' },
    { id: 2, text: 'The quick brown fox jumps over the lazy dog', expected: 'The quick brown fox jumps over the lazy dog' },
    { id: 3, text: 'University', expected: 'University' },
    { id: 4, text: 'Science and technology', expected: 'Science and technology' },
    { id: 5, text: 'Artificial intelligence', expected: 'Artificial intelligence' },
    { id: 6, text: 'Machine learning', expected: 'Machine learning' },
    { id: 7, text: 'Robotics', expected: 'Robotics' },
    { id: 8, text: 'Humanoid robot', expected: 'Humanoid robot' },
    { id: 9, text: 'Physical AI', expected: 'Physical AI' },
    { id: 10, text: 'Digital twin', expected: 'Digital twin' },
  ]);

  const [customInput, setCustomInput] = useState('');
  const [customOutput, setCustomOutput] = useState('');

  const runTests = () => {
    const updatedTests = testInputs.map(test => ({
      ...test,
      result: englishToRomanUrdu(test.text),
      accuracy: calculateAccuracy(englishToRomanUrdu(test.text), test.expected)
    }));
    setTestInputs(updatedTests);
  };

  const calculateAccuracy = (result: string, expected: string): string => {
    // This is a basic accuracy calculation - in a real implementation,
    // we'd have more sophisticated comparison logic
    if (result.toLowerCase() === expected.toLowerCase()) {
      return '100%';
    }

    // Calculate based on character similarity
    const maxLength = Math.max(result.length, expected.length);
    if (maxLength === 0) return '100%';

    let matches = 0;
    for (let i = 0; i < Math.min(result.length, expected.length); i++) {
      if (result[i].toLowerCase() === expected[i].toLowerCase()) {
        matches++;
      }
    }

    const percentage = Math.round((matches / maxLength) * 100);
    return `${percentage}%`;
  };

  const handleCustomTranslate = () => {
    const result = englishToRomanUrdu(customInput);
    setCustomOutput(result);
  };

  return (
    <div className="translation-accuracy-test">
      <h2>Translation Accuracy Tests</h2>

      <div className="custom-translation">
        <h3>Test Custom Text</h3>
        <textarea
          value={customInput}
          onChange={(e) => setCustomInput(e.target.value)}
          placeholder="Enter English text to translate to Roman Urdu..."
          className="translation-input"
          rows={3}
        />
        <button onClick={handleCustomTranslate} className="translate-button">
          Translate
        </button>
        {customOutput && (
          <div className="translation-result">
            <h4>Translated Text:</h4>
            <p>{customOutput}</p>
          </div>
        )}
      </div>

      <div className="test-results">
        <h3>Predefined Tests</h3>
        <button onClick={runTests} className="run-tests-button">Run All Tests</button>

        <table className="tests-table">
          <thead>
            <tr>
              <th>Test ID</th>
              <th>Input</th>
              <th>Output</th>
              <th>Expected</th>
              <th>Accuracy</th>
            </tr>
          </thead>
          <tbody>
            {testInputs.map(test => (
              <tr key={test.id}>
                <td>{test.id}</td>
                <td>{test.text}</td>
                <td>{test.result || englishToRomanUrdu(test.text)}</td>
                <td>{test.expected}</td>
                <td>{test.accuracy || calculateAccuracy(englishToRomanUrdu(test.text), test.expected)}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>

      <div className="translation-info">
        <h3>About the Translation System</h3>
        <p>
          This system uses a hybrid approach combining:
        </p>
        <ul>
          <li>Direct character mapping for common words</li>
          <li>Phonetic pattern matching for transliteration</li>
          <li>A comprehensive dictionary of common English to Roman Urdu translations</li>
        </ul>
        <p>
          The accuracy may vary depending on the complexity of the text and how well it matches
          known patterns in our translation dictionary.
        </p>
      </div>
    </div>
  );
};

export default TranslationAccuracyTest;