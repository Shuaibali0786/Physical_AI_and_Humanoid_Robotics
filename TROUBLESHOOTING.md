# Troubleshooting Guide

This guide helps troubleshoot common issues with the Qwen RAG chatbot functionality.

## Issue: "Sorry, there was an error processing your request"

### Possible Causes and Solutions:

#### 1. Qwen API Server Not Running
**Symptoms:** The error message appears immediately when sending a message.

**Solution:**
- Make sure the Qwen API server is running:
  ```bash
  cd backend/qwen_api
  python -m uvicorn main:app --host 0.0.0.0 --port 8000
  ```

#### 2. Port Conflict
**Symptoms:** Server fails to start with port binding errors.

**Solution:**
- Check if port 8000 is already in use:
  ```bash
  # On Windows
  netstat -ano | findstr :8000
  ```
- If port 8000 is in use, kill the process or use a different port.

#### 3. Network/Firewall Issues
**Symptoms:** Frontend can't connect to backend server.

**Solution:**
- Check if you can access the test endpoint:
  - Open browser and go to `http://localhost:8000/api/test`
  - You should see a JSON response: `{"message": "Qwen RAG API is running correctly", "status": "ok"}`

#### 4. Missing Dependencies
**Symptoms:** Server fails to start with import errors.

**Solution:**
- Install all required dependencies:
  ```bash
  cd backend/qwen_api
  pip install -r requirements.txt
  ```

## Testing the API Server

Run the test script to verify the API server is working:

```bash
cd backend/qwen_api
python test_api.py
```

## Step-by-Step Setup

1. **Start the Qwen API server:**
   ```bash
   cd backend/qwen_api
   python -m uvicorn main:app --host 0.0.0.0 --port 8000
   ```

2. **In a new terminal, start the frontend:**
   ```bash
   npm start
   ```

3. **Test the connection:**
   - Open browser to `http://localhost:3000`
   - Click the floating chat button
   - Type "hi" and send the message
   - You should receive a response

## Common Error Messages

- **"Network error. Please make sure the Qwen API server is running"**: The API server is not running or not accessible at http://localhost:8000
- **"Error processing your request"**: The API server received the request but failed to process it
- **"Error: Not Found"**: This typically means the API endpoint doesn't exist or the server isn't running properly
- **CORS errors**: Usually indicates the API server is running but on a different port

## Debugging Steps for "Error: Not Found"

1. **Check if the Qwen API server is running**:
   - Open a terminal and run:
   ```bash
   cd backend/qwen_api
   python -m uvicorn main:app --host 0.0.0.0 --port 8000
   ```
   - You should see a message like: "Uvicorn running on http://0.0.0.0:8000"

2. **Verify the API endpoints are accessible**:
   - Open your browser and go to: `http://localhost:8000/health`
   - You should see: `{"status": "healthy", "service": "Qwen RAG Chat API"}`
   - Also check: `http://localhost:8000/api/test`
   - You should see: `{"message": "Qwen RAG API is running correctly", "status": "ok"}`

3. **Check the browser console for errors**:
   - Press F12 in your browser
   - Look at the Console tab for any error messages
   - Look for network errors related to http://localhost:8000

4. **Check the Network tab**:
   - In browser developer tools, go to the Network tab
   - Send a message in the chat
   - Check if the request to `http://localhost:8000/api/qwen-chat` is showing as 404 (Not Found) or another error

5. **Run the test script**:
   ```bash
   cd backend/qwen_api
   python test_api.py
   ```

6. **Verify your .env file**:
   - Make sure your `.env` file in the `backend/qwen_api` directory contains all the required configuration
   - Check that the QWEN_API_KEY is properly set (replace "your_actual_qwen_api_key_here" with your real API key)

7. **Check if you have installed all dependencies**:
   ```bash
   cd backend/qwen_api
   pip install -r requirements.txt
   ```

8. **Restart both servers**:
   - Stop both the frontend and backend servers
   - Start the backend first: `cd backend/qwen_api && python -m uvicorn main:app --host 0.0.0.0 --port 8000`
   - Then start the frontend: `npm start`

9. **If you don't have a Qwen API key yet**, the system will use fallback responses that should still work for basic queries like "hi" and "what is ai".