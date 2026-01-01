# Script to verify that all required API endpoints are properly defined
import sys
import os

# Add the current directory to Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from main import app
    from fastapi.testclient import TestClient

    client = TestClient(app)

    print("Verifying API endpoints...")

    # Test health endpoint
    response = client.get("/health")
    if response.status_code == 200:
        print("✓ Health endpoint: OK")
        print(f"  Response: {response.json()}")
    else:
        print(f"✗ Health endpoint: Error {response.status_code}")

    # Test test endpoint
    response = client.get("/api/test")
    if response.status_code == 200:
        print("✓ Test endpoint: OK")
        print(f"  Response: {response.json()}")
    else:
        print(f"✗ Test endpoint: Error {response.status_code}")

    # Test qwen-chat endpoint (should return 422 for missing body, but route should exist)
    response = client.post("/api/qwen-chat")
    if response.status_code in [422, 405]:  # 422 = validation error, 405 = method not allowed
        print("✓ Qwen chat endpoint: Route exists (expected validation error without proper request body)")
    elif response.status_code == 404:
        print("✗ Qwen chat endpoint: NOT FOUND - This is the issue!")
    else:
        print(f"? Qwen chat endpoint: Status {response.status_code}")

    print("\nEndpoint verification complete!")

except ImportError as e:
    print(f"Error importing app: {e}")
    print("Make sure you're in the correct directory and have FastAPI installed")
except Exception as e:
    print(f"Error during verification: {e}")