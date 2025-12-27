@echo off
echo Starting Qwen RAG API Server...
echo.
echo Please make sure you have installed the required dependencies:
echo cd backend\qwen_api
echo pip install -r requirements.txt
echo.
echo Press any key to start the API server on port 8000...
pause >nul

cd /d "E:\physical_ai_text_book\Physical_AI_and_Humanoid_Robotics\backend\qwen_api"
echo Starting server...
python -m uvicorn main:app --host 0.0.0.0 --port 8000