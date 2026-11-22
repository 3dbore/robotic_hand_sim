"""
Entry point for running the ELIOS HIL Robot Arm WebApp Backend server.
"""
import uvicorn

if __name__ == "__main__":
    uvicorn.run("backend.server:app", host="127.0.0.1", port=8000, reload=False)
