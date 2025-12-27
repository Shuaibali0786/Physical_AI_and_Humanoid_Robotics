@echo off
echo Starting Qwen RAG API server...
cd /d "E:\physical_ai_text_book\Physical_AI_and_Humanoid_Robotics\backend\qwen_api"
python -m uvicorn main:app --host 0.0.0.0 --port 8000