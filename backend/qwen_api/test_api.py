# Test script to verify the Qwen API server is running
import requests
import sys

def test_api_connection():
    try:
        # Test the health endpoint
        response = requests.get('http://localhost:8000/health')
        if response.status_code == 200:
            print("✓ Health check passed:", response.json())
        else:
            print("✗ Health check failed with status code:", response.status_code)
            return False

        # Test the test endpoint
        response = requests.get('http://localhost:8000/api/test')
        if response.status_code == 200:
            print("✓ Test endpoint passed:", response.json())
        else:
            print("✗ Test endpoint failed with status code:", response.status_code)
            return False

        print("\n✓ API server is running correctly!")
        print("You can now start the Docusaurus frontend with 'npm start'")
        print("The chat should work properly once both servers are running.")
        return True

    except requests.exceptions.ConnectionError:
        print("✗ Cannot connect to the API server at http://localhost:8000")
        print("Please make sure you've started the Qwen API server with:")
        print("cd backend/qwen_api")
        print("python -m uvicorn main:app --host 0.0.0.0 --port 8000")
        return False
    except Exception as e:
        print(f"✗ Error testing API connection: {str(e)}")
        return False

if __name__ == "__main__":
    test_api_connection()