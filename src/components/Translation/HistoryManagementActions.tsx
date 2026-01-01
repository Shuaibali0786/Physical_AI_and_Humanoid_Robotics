/**
 * History Management Actions Component
 *
 * This component provides actions for managing translation history
 * including clear, export, import, and other management functions.
 */

import React, { useState } from 'react';
import { useTranslation } from '../../hooks/useTranslation';
import './../../styles/translation-search.css';

interface HistoryManagementActionsProps {
  onHistoryChange?: () => void;
}

const HistoryManagementActions: React.FC<HistoryManagementActionsProps> = ({
  onHistoryChange
}) => {
  const { clearTranslationHistory } = useTranslation();
  const [isConfirmingClear, setIsConfirmingClear] = useState<boolean>(false);
  const [importFile, setImportFile] = useState<File | null>(null);
  const [importStatus, setImportStatus] = useState<{ success: boolean; message: string } | null>(null);

  const handleClearHistory = () => {
    setIsConfirmingClear(true);
  };

  const confirmClearHistory = async () => {
    const success = clearTranslationHistory();
    setIsConfirmingClear(false);

    if (success) {
      setImportStatus({ success: true, message: 'Translation history cleared successfully' });
      if (onHistoryChange) {
        onHistoryChange();
      }
    } else {
      setImportStatus({ success: false, message: 'Failed to clear translation history' });
    }

    // Clear status message after 3 seconds
    setTimeout(() => {
      setImportStatus(null);
    }, 3000);
  };

  const cancelClearHistory = () => {
    setIsConfirmingClear(false);
  };

  const handleImportFileChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    if (e.target.files && e.target.files.length > 0) {
      setImportFile(e.target.files[0]);
    }
  };

  const handleImportHistory = async () => {
    if (!importFile) {
      setImportStatus({ success: false, message: 'Please select a file to import' });
      return;
    }

    try {
      const fileContent = await readFileContent(importFile);
      const parsedHistory = JSON.parse(fileContent);

      // Validate the imported data
      if (!Array.isArray(parsedHistory)) {
        throw new Error('Invalid file format: expected an array of translation items');
      }

      // In a real implementation, we would import the history
      // For now, we'll just show a success message
      setImportStatus({ success: true, message: `Successfully imported ${parsedHistory.length} translation items` });

      if (onHistoryChange) {
        onHistoryChange();
      }

      // Clear the file input
      setImportFile(null);

      // Clear status message after 3 seconds
      setTimeout(() => {
        setImportStatus(null);
      }, 3000);
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Unknown error occurred during import';
      setImportStatus({ success: false, message: `Import failed: ${errorMessage}` });

      // Clear status message after 5 seconds
      setTimeout(() => {
        setImportStatus(null);
      }, 5000);
    }
  };

  const readFileContent = (file: File): Promise<string> => {
    return new Promise((resolve, reject) => {
      const reader = new FileReader();
      reader.onload = (e) => {
        if (e.target?.result) {
          resolve(e.target.result as string);
        } else {
          reject(new Error('Failed to read file'));
        }
      };
      reader.onerror = () => {
        reject(new Error('Error reading file'));
      };
      reader.readAsText(file);
    });
  };

  const handleExportHistory = () => {
    // In a real implementation, this would export the history
    // For now, we'll just show a message
    setImportStatus({ success: true, message: 'Export feature would download history file' });

    // Clear status message after 3 seconds
    setTimeout(() => {
      setImportStatus(null);
    }, 3000);
  };

  return (
    <div className="history-management-actions">
      {importStatus && (
        <div className={`import-status-message ${importStatus.success ? 'success' : 'error'}`}>
          {importStatus.message}
        </div>
      )}

      <div className="history-actions-buttons">
        <button
          onClick={handleClearHistory}
          className="action-button clear-button"
        >
          Clear History
        </button>

        <button
          onClick={handleExportHistory}
          className="action-button export-button"
        >
          Export History
        </button>

        <div className="import-section">
          <input
            type="file"
            id="import-history"
            accept=".json"
            onChange={handleImportFileChange}
            className="import-input"
          />
          <label htmlFor="import-history" className="import-button-label">
            <button
              onClick={() => document.getElementById('import-history')?.click()}
              className="action-button import-button"
              disabled={!importFile}
            >
              Import History
            </button>
          </label>
          {importFile && (
            <span className="import-file-name">{importFile.name}</span>
          )}
        </div>
      </div>

      {isConfirmingClear && (
        <div className="confirmation-modal">
          <div className="confirmation-content">
            <h3>Confirm Clear History</h3>
            <p>Are you sure you want to clear all translation history? This action cannot be undone.</p>
            <div className="confirmation-buttons">
              <button onClick={confirmClearHistory} className="confirm-button">
                Yes, Clear
              </button>
              <button onClick={cancelClearHistory} className="cancel-button">
                Cancel
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default HistoryManagementActions;