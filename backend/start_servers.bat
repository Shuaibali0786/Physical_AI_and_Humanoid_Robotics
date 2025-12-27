@echo off
REM Startup script to run both the Qwen API server and the Search/Translation API server

echo Starting Qwen RAG API server and Search/Translation API server...

REM Start the Qwen API server on port 8000 in a separate window
start "Qwen API Server" cmd /k "cd /d "%~dp0qwen_api" && python main.py"

REM Wait a moment for the first server to start
timeout /t 3 /nobreak >nul

REM Start the Search/Translation API server on port 8001 in another separate window
start "Search/Translation API Server" cmd /k "cd /d "%~dp0" && python api.py"

echo Both servers started successfully!
echo Qwen API server running on http://localhost:8000
echo Search/Translation API server running on http://localhost:8001
echo.
echo Press Ctrl+C in each window to stop the servers.
pause