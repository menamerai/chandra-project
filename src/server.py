import os
import tempfile
from parser import command_parsing

import uvicorn
import whisper
from fastapi import Body, FastAPI, File, HTTPException, UploadFile, APIRouter
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import RedirectResponse
from loguru import logger

from brigde import Direction, velocity_pub

app = FastAPI(title="Robot Command Service")
turtle_router = APIRouter(prefix="/turtle", tags=["Turtlebot"])
cheetah_router = APIRouter(prefix="/cheetah", tags=["Cheetah"])

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
    global model
    try:
        model = whisper.load_model("large-v3-turbo")
        logger.info("Whisper model loaded successfully")
    except Exception as e:
        logger.error(f"Error loading Whisper model: {str(e)}")
        # Continue anyway, will try to load again when needed


# Legacy endpoint for backward compatibility
@app.post("/velocity", deprecated=True)
async def publish_velocity_legacy(
    direction: Direction | str = Body(...), execution_time: int = Body(...)
):
    """
    Legacy endpoint. Use /turtle/velocity or /cheetah/velocity instead.
    """
    try:
        velocity_pub(direction=direction, execution_time=execution_time, robot_type="turtlebot")
        return {"message": "Velocity command published successfully (using turtlebot)"}
    except Exception as e:
        logger.error(f"Error publishing velocity command: {str(e)}")
        raise HTTPException(
            status_code=500, detail=f"Error publishing velocity command: {str(e)}"
        )


# Legacy endpoint for backward compatibility
@app.post("/command", deprecated=True)
async def transcribe_command_legacy(file: UploadFile = File(...)):
    """
    Legacy endpoint. Use /turtle/command or /cheetah/command instead.
    """
    return await transcribe_audio_file(file, "turtlebot")


# Helper function for audio transcription
async def transcribe_audio_file(file: UploadFile, robot_type: str):
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
            model = whisper.load_model("large-v3")

        # Transcribe the audio
        result = model.transcribe(temp_path)
        text = result["text"]
        command = command_parsing(text)
        logger.info(f"Audio transcription completed successfully for {robot_type}")

        # Clean up temporary file
        os.unlink(temp_path)

        # Return the transcribed text
        return {"text": text, "command": command, "robot_type": robot_type}

    except Exception as e:
        # Make sure to clean up if there's an error
        if "temp_path" in locals() and os.path.exists(temp_path):
            os.unlink(temp_path)
        logger.error(f"Error processing audio: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing audio: {str(e)}")


# Turtlebot Endpoints
@turtle_router.post("/velocity")
async def publish_turtle_velocity(
    direction: Direction | str = Body(...), execution_time: int = Body(...)
):
    """
    Publishes a velocity command to the Turtlebot robot.
    """
    try:
        velocity_pub(direction=direction, execution_time=execution_time, robot_type="turtlebot")
        return {"message": "Turtlebot velocity command published successfully"}
    except Exception as e:
        logger.error(f"Error publishing Turtlebot velocity command: {str(e)}")
        raise HTTPException(
            status_code=500, detail=f"Error publishing Turtlebot velocity command: {str(e)}"
        )


@turtle_router.post("/command")
async def transcribe_turtle_command(file: UploadFile = File(...)):
    """
    Accepts a .wav audio file and returns its transcription for Turtlebot commands.
    """
    return await transcribe_audio_file(file, "turtlebot")


# Cheetah Endpoints
@cheetah_router.post("/velocity")
async def publish_cheetah_velocity(
    direction: Direction | str = Body(...), execution_time: int = Body(...)
):
    """
    Publishes a velocity command to the Cheetah robot.
    """
    try:
        velocity_pub(direction=direction, execution_time=execution_time, robot_type="cheetah")
        return {"message": "Cheetah velocity command published successfully"}
    except Exception as e:
        logger.error(f"Error publishing Cheetah velocity command: {str(e)}")
        raise HTTPException(
            status_code=500, detail=f"Error publishing Cheetah velocity command: {str(e)}"
        )


@cheetah_router.post("/command")
async def transcribe_cheetah_command(file: UploadFile = File(...)):
    """
    Accepts a .wav audio file and returns its transcription for Cheetah commands.
    """
    return await transcribe_audio_file(file, "cheetah")


# Include routers in the main app
app.include_router(turtle_router)
app.include_router(cheetah_router)


if __name__ == "__main__":
    logger.info("Starting Robot Command Service")
    uvicorn.run(app, host="0.0.0.0", port=8000)
