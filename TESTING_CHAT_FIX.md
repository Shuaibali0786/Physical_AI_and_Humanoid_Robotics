# Testing the Fixed Chat Functionality

The issue where the frontend chat was showing but not responding to "hi" has been resolved. Here's how to verify:

## Step 1: Verify Dependencies

1. Make sure you have Python installed on your system
2. Navigate to the backend/qwen_api directory:
   ```bash
   cd backend\qwen_api
   ```
3. Install required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Step 2: Start the Qwen API Server

1. Open a terminal/command prompt
2. Navigate to the backend/qwen_api directory:
   ```bash
   cd backend\qwen_api
   ```
3. Start the server:
   ```bash
   python -m uvicorn main:app --host 0.0.0.0 --port 8000
   ```
4. Verify the server is running by visiting: http://localhost:8000/health
   - You should see: `{"status": "healthy", "service": "Qwen RAG Chat API"}`
5. Also test: http://localhost:8000/api/test
   - You should see: `{"message": "Qwen RAG API is running correctly", "status": "ok"}`

## Step 3: Start the Docusaurus Frontend

1. In a new terminal/command prompt, navigate to the main project directory
2. Start the Docusaurus development server:
   ```bash
   npm start
   ```

## Step 4: Test the Chat Functionality

1. Open your browser and navigate to the Docusaurus site (usually http://localhost:3000)
2. Look for the floating chat button (should be in the bottom-right corner)
3. Click the chat button to open the chat interface
4. Type "hi" or "hello" in the input field
5. Click "Send" or press Enter

## Expected Results

- When you type "hi" or "hello", the assistant should respond with something like: "Hello! I'm your Qwen RAG Assistant. I can help you with questions about the book content. What would you like to know?"
- The chat interface should now properly communicate with the backend API
- The response should appear in the chat window

## Troubleshooting

If you still see "Sorry, there was an error processing your request":

1. **Check that the Qwen API server is running on port 8000**
   - Look for the message in the server terminal: "Uvicorn running on http://0.0.0.0:8000"
   - If not running, start it with: `python -m uvicorn main:app --host 0.0.0.0 --port 8000`

2. **Check the browser console for errors**
   - Press F12 in your browser
   - Look at the Console tab for any error messages
   - Look for network errors related to http://localhost:8000

3. **Check the server terminal for error messages**
   - Look for any error messages when the frontend makes requests

4. **Verify the API endpoints are accessible**
   - Open http://localhost:8000/health in your browser
   - Open http://localhost:8000/api/test in your browser

5. **Run the test script**
   ```bash
   cd backend/qwen_api
   python test_api.py
   ```

6. **Make sure you have the required dependencies installed**
   ```bash
   cd backend/qwen_api
   pip install -r requirements.txt
   ```

7. **If using a virtual environment**, make sure it's activated:
   ```bash
   python -m venv venv
   venv\Scripts\activate  # On Windows
   pip install -r requirements.txt
   ```

## For Development

For a better development experience, you can copy the .env.example file and add your API keys:
```bash
cd backend/qwen_api
copy .env.example .env
# Then edit .env with your actual API keys
```

If you don't have API keys yet, the system will use mock responses which should still allow you to test the basic functionality.