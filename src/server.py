import os
import tempfile
import sys

# add src to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))

from parser import command_parsing

import uvicorn
from faster_whisper import BatchedInferencePipeline, WhisperModel
from fastapi import FastAPI, File, HTTPException, UploadFile, APIRouter
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import RedirectResponse
from loguru import logger
from pydantic import BaseModel

# Replace the direct import with the subprocess bridge
from mavlink_wrapper import GhostRobotClientWrapper as GhostRobotClient

app = FastAPI(title="Robot Command Service")
# Initialize robot client with subprocess bridge to Python 3.8
v60 = GhostRobotClient(sim=True)
logger.info("Robot client initialized with Python 3.8 bridge")

class ActionRequest(BaseModel):
    mode: int  # 0=sit, 1=stand, 2=walk

class MovementRequest(BaseModel):
    x: float = 0.0  # Forward/backward (-1.0 to 1.0)
    y: float = 0.0  # Left/right (-1.0 to 1.0)
    z: float = 0.0  # Rotation (-1.0 to 1.0)
    r: float = 0.0  # Height adjustment (-1.0 to 1.0)
    duration: float = 0.0  # Duration of movement (0 = immediate)

# Create a router for V60 endpoints
robot_router = APIRouter(prefix="/robot", tags=["robot"])

# Set up CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Adjust in production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Redirect root to /docs
@app.get("/", include_in_schema=False)
async def root():
    return RedirectResponse(url="/docs")

# Load Whisper model once at startup to avoid reloading it for each request
@app.on_event("startup")
async def startup_event():
    global model, model_name
    model_name = "distil-large-v3"
    # model_name = "tiny"
    try:
        # Try GPU first
        model = BatchedInferencePipeline(WhisperModel(model_name, device="cuda", compute_type="float16"))
        logger.info("Whisper model loaded successfully on GPU")
    except Exception as e:
        logger.warning(f"GPU loading failed, falling back to CPU: {str(e)}")
        try:
            model = BatchedInferencePipeline(WhisperModel(model_name, device="cpu", compute_type="int8"))
            logger.info("Whisper model loaded successfully on CPU")
        except Exception as e:
            logger.error(f"Error loading Whisper model: {str(e)}")
            # Continue anyway, will try to load again when needed

@app.on_event("shutdown")
def shutdown_event():
    """Clean up robot connection when shutting down server"""
    v60.close()
    GhostRobotClient.shutdown_daemon()
    logger.info("Robot connection closed")

@app.post("/command", deprecated=False)
async def transcribe_command_legacy(file: UploadFile = File(...)):
    """
    Transcribe audio file and return commands.
    """
    return await transcribe_audio_file(file)

# Helper function for audio transcription
async def transcribe_audio_file(file: UploadFile):
    """
    Helper function to transcribe audio files and return commands.
    """
    if not file.filename.lower().endswith(".wav"):
        logger.warning(f"Rejected file with unsupported format: {file.filename}")
        raise HTTPException(status_code=400, detail="Only .wav files are supported")

    try:
        # Create a temporary file to save the uploaded audio
        with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as temp_file:
            temp_path = temp_file.name
            # Write uploaded file content to temporary file
            temp_file.write(await file.read())

        logger.debug(f"Audio saved to temporary file: {temp_path}")

        # Ensure model is available
        if "model" not in globals():
            global model
            logger.info("Loading Whisper model on demand")
            try:
                logger.info("Attempting to load Whisper model on GPU")
                model = BatchedInferencePipeline(WhisperModel(model_name, device="cuda", compute_type="float16"))
            except Exception as e:
                logger.warning(f"GPU loading failed, falling back to CPU: {str(e)}")
                try:
                    model = BatchedInferencePipeline(WhisperModel(model_name, device="cpu", compute_type="int8"))
                except Exception as e:
                    logger.error(f"Error loading Whisper model: {str(e)}")
                    raise HTTPException(status_code=500, detail="Error loading Whisper model")

        # Transcribe the audio
        segments, _ = model.transcribe(temp_path, beam_size=5, language="en", task="transcribe")
        segments = list(segments)

        text = " ".join([segment.text for segment in segments])
        logger.debug(f"Transcribed text: {text}")

        # Parse the command from the transcribed text
        command = command_parsing(text)
        logger.debug(f"Parsed command: {command}")

        # Clean up temporary file
        os.unlink(temp_path)

        # Return the transcribed text
        return {"text": text, "command": command }

    except Exception as e:
        # Make sure to clean up if there's an error
        if "temp_path" in locals() and os.path.exists(temp_path):
            os.unlink(temp_path)
        logger.error(f"Error processing audio: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing audio: {str(e)}")

@robot_router.post("/action")
async def set_action_mode(request: ActionRequest):
    """Set robot action mode: 0=sit, 1=stand, 2=walk"""
    if request.mode not in [0, 1, 2]:
        raise HTTPException(status_code=400, detail="Mode must be 0 (sit), 1 (stand), or 2 (walk)")
    try:
        current_mode = v60.set_action_mode(request.mode)
        return {"success": True, "requested_mode": request.mode, "current_mode": current_mode}
    except Exception as e:
        logger.error(f"Error setting action mode: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error setting action mode: {str(e)}")

@robot_router.post("/move")
async def move_robot(request: MovementRequest):
    """Move robot with specified velocity components"""
    try:
        success = v60.move_robot(
            x=request.x, 
            y=request.y, 
            z=request.z, 
            r=request.r,
            duration=request.duration
        )
        if not success:
            raise HTTPException(status_code=400, detail="Failed to move robot. Check robot mode.")
        return {
            "success": True, 
            "movement": {
                "x": request.x, 
                "y": request.y, 
                "z": request.z, 
                "r": request.r,
                "duration": request.duration
            }
        }
    except Exception as e:
        logger.error(f"Error moving robot: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error moving robot: {str(e)}")

@robot_router.post("/stop")
async def stop_robot():
    """Stop all robot movement"""
    try:
        success = v60.stop_movement()
        return {"success": success, "message": "Robot stopped"}
    except Exception as e:
        logger.error(f"Error stopping robot: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error stopping robot: {str(e)}")
    
@robot_router.post("/roll_over")
async def roll_over():
    """Make the robot roll over"""
    try:
        success = v60.roll_over(1)
        return {"success": success, "message": "Robot rolled over"}
    except Exception as e:
        logger.error(f"Error rolling over robot: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error rolling over robot: {str(e)}")

@robot_router.post("/status")
async def get_status():
    """Get robot status information"""
    status = v60.get_status()
    # Convert numpy arrays to lists for JSON serialization
    clean_status = {}
    for key, value in status.items():
        if hasattr(value, 'tolist'):
            clean_status[key] = value.tolist()
        else:
            clean_status[key] = value
    return {"status": clean_status}

# Include the v60 router in the main app
app.include_router(robot_router)

if __name__ == "__main__":
    logger.info("Starting Robot Command Service")
    uvicorn.run(app, host="0.0.0.0", port=8000)
