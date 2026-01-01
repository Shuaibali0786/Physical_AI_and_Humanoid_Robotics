# Physical AI and Humanoid Robotics

This project contains a digital book on Physical AI and Humanoid Robotics with integrated AI-powered features including a RAG chatbot powered by Qwen.

## Project Structure

- `docs/` - Book content and documentation
- `src/` - Frontend source code (Docusaurus theme components)
- `backend/` - Backend services
  - `qwen_api/` - Qwen-powered RAG chatbot API
- `specs/` - Specification files for various features

## Running the Application

### Frontend (Docusaurus)

1. Install dependencies:
   ```bash
   npm install
   ```

2. Start the development server:
   ```bash
   npm start
   ```

### Backend Services

#### Qwen RAG Chat API

1. Navigate to the Qwen API directory:
   ```bash
   cd backend/qwen_api
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Configure your environment:
   - Copy the example environment file: `copy .env.example .env` (on Windows) or `cp .env.example .env` (on macOS/Linux)
   - Edit the `.env` file with your API keys and configuration
   - Make sure to add your Qwen API key to the `QWEN_API_KEY` variable

4. Start the API server:
   ```bash
   python -m uvicorn main:app --host 0.0.0.0 --port 8000
   ```

The frontend expects the Qwen API to be available at `http://localhost:8000`.

## Features

- Interactive chatbot powered by Qwen for questions about the book content
- Translation functionality (English to Roman Urdu)
- Search functionality across book modules
- Responsive design for all device sizes