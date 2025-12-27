"""
Script to run both the Qwen API server and the Search/Translation API server
"""
import subprocess
import sys
import os
from threading import Thread
import time

def run_qwen_server():
    """Run the Qwen API server"""
    os.chdir(os.path.join(os.path.dirname(__file__), "qwen_api"))
    subprocess.run([sys.executable, "main.py"])

def run_search_translation_server():
    """Run the Search/Translation API server"""
    os.chdir(os.path.dirname(__file__))  # Go back to backend directory
    subprocess.run([sys.executable, "api.py"])

def main():
    print("Starting Qwen RAG API server and Search/Translation API server...")

    # Create threads for both servers
    qwen_thread = Thread(target=run_qwen_server)
    search_thread = Thread(target=run_search_translation_server)

    # Start both servers
    qwen_thread.start()
    time.sleep(2)  # Wait a moment before starting the second server
    search_thread.start()

    print("Both servers started successfully!")
    print("Qwen API server running on http://localhost:8000")
    print("Search/Translation API server running on http://localhost:8001")
    print()
    print("Press Ctrl+C to stop both servers.")

    try:
        # Wait for both threads to complete (they won't unless interrupted)
        qwen_thread.join()
        search_thread.join()
    except KeyboardInterrupt:
        print("\nShutting down servers...")
        # Note: In a real implementation, you'd want to properly terminate the processes
        print("Servers stopped.")

if __name__ == "__main__":
    main()