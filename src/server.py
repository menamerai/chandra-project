import os
import tempfile
import sys
import time
import asyncio

# add src to path
sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
)

from parser import command_parsing

import uvicorn
from faster_whisper import BatchedInferencePipeline, WhisperModel
from fastapi import FastAPI, File, HTTPException, UploadFile, APIRouter, Body
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import RedirectResponse
from loguru import logger
from typing import Optional
from agents import CommandType, agentic_process

# Replace the direct import with the subprocess bridge
from mavlink_wrapper import GhostRobotClientWrapper as GhostRobotClient

app = FastAPI(title="Robot Command Service")
# Initialize robot client with subprocess bridge to Python 3.8
v60 = GhostRobotClient(sim=True)
logger.info("Robot client initialized with Python 3.8 bridge")


# Create a router for V60 endpoints
audio_router = APIRouter(prefix="/audio", tags=["audio"])
agent_router = APIRouter(prefix="/agent", tags=["agent"])
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
        model = BatchedInferencePipeline(
            WhisperModel(model_name, device="cuda", compute_type="float16")
        )
        logger.info("Whisper model loaded successfully on GPU")
    except Exception as e:
        logger.warning(f"GPU loading failed, falling back to CPU: {str(e)}")
        try:
            model = BatchedInferencePipeline(
                WhisperModel(model_name, device="cpu", compute_type="int8")
            )
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


@audio_router.post("/command", deprecated=False)
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
                model = BatchedInferencePipeline(
                    WhisperModel(model_name, device="cuda", compute_type="float16")
                )
            except Exception as e:
                logger.warning(f"GPU loading failed, falling back to CPU: {str(e)}")
                try:
                    model = BatchedInferencePipeline(
                        WhisperModel(model_name, device="cpu", compute_type="int8")
                    )
                except Exception as e:
                    logger.error(f"Error loading Whisper model: {str(e)}")
                    raise HTTPException(
                        status_code=500, detail="Error loading Whisper model"
                    )

        # Transcribe the audio
        segments, _ = model.transcribe(
            temp_path, beam_size=5, language="en", task="transcribe"
        )
        segments = list(segments)

        text = " ".join([segment.text for segment in segments])
        logger.debug(f"Transcribed text: {text}")

        # Parse the command from the transcribed text
        command = command_parsing(text)
        logger.debug(f"Parsed command: {command}")

        # Clean up temporary file
        os.unlink(temp_path)

        # Return the transcribed text
        return {"text": text, "command": command}

    except Exception as e:
        # Make sure to clean up if there's an error
        if "temp_path" in locals() and os.path.exists(temp_path):
            os.unlink(temp_path)
        logger.error(f"Error processing audio: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing audio: {str(e)}")


@agent_router.post("/agent_run")
async def agent_process_text(payload: dict = Body(...)):
    text = payload.get("text")
    if not text:
        raise HTTPException(status_code=400, detail="Text is required")
    try:
        # Trigger the agentic process
        await asyncio.to_thread(agentic_process, text)
        return {"message": "Agent process triggered successfully"}
    except Exception as e:
        logger.error(f"Error processing agent request: {str(e)}")
        raise HTTPException(
            status_code=500, detail=f"Error processing agent request: {str(e)}"
        )


@robot_router.post("/commands")
async def execute_command(
    commands: list[tuple[CommandType | str, Optional[float]]] = [
        (CommandType.MOVE_FORWARD, 2.0),
        (CommandType.MOVE_BACKWARD, 2.0),
        (CommandType.MOVE_LEFT, 2.0),
        (CommandType.MOVE_RIGHT, 2.0),
        (CommandType.MOVE_FORWARD_LEFT, 1.0),
        (CommandType.MOVE_FORWARD_RIGHT, 1.0),
        (CommandType.MOVE_BACKWARD_LEFT, 1.0),
        (CommandType.MOVE_BACKWARD_RIGHT, 1.0),
        (CommandType.ROTATE_LEFT, 2.0),
        (CommandType.ROTATE_RIGHT, 2.0),
        (CommandType.SIT, 1.0),
        (CommandType.STAND, 1.0),
        (CommandType.ROLL_OVER, 1.0),
        (CommandType.STOP, 3.0),
    ]
):
    """
    Execute a series of commands on the robot.
    Commands can include movement and action commands.

    Args:
        commands (list): A list of tuples where each tuple contains a command type and optional duration.

    """
    try:
        for command in commands:
            logger.debug(f"Executing command: {command}")
            cmd_type = command[0]
            if isinstance(cmd_type, str):
                cmd_type = CommandType(cmd_type)
            duration = command[1] if len(command) > 1 else None

            match cmd_type:
                case CommandType.MOVE_FORWARD:
                    v60.move_robot(x=1.0, duration=duration)
                case CommandType.MOVE_BACKWARD:
                    v60.move_robot(x=-1.0, duration=duration)
                case CommandType.MOVE_LEFT:
                    v60.move_robot(y=-1.0, duration=duration)
                case CommandType.MOVE_RIGHT:
                    v60.move_robot(y=1.0, duration=duration)
                case CommandType.MOVE_FORWARD_LEFT:
                    v60.move_robot(x=1.0, y=-1.0, duration=duration)
                case CommandType.MOVE_FORWARD_RIGHT:
                    v60.move_robot(x=1.0, y=1.0, duration=duration)
                case CommandType.MOVE_BACKWARD_LEFT:
                    v60.move_robot(x=-1.0, y=-1.0, duration=duration)
                case CommandType.MOVE_BACKWARD_RIGHT:
                    v60.move_robot(x=-1.0, y=1.0, duration=duration)
                case CommandType.ROTATE_LEFT:
                    v60.move_robot(z=-1.0, duration=duration)
                case CommandType.ROTATE_RIGHT:
                    v60.move_robot(z=1.0, duration=duration)
                case CommandType.SIT:
                    v60.set_action_mode(0)
                    time.sleep(duration if duration >= 3 else 3)
                case CommandType.STAND:
                    v60.set_action_mode(2)
                    time.sleep(duration if duration >= 3 else 3)
                case CommandType.ROLL_OVER:
                    v60.roll_over()
                    time.sleep(duration if duration >= 8 else 8)
                case CommandType.STOP:
                    v60.stop_movement()
                    time.sleep(duration if duration else 0)
                case _:
                    logger.warning(f"Unknown command type: {cmd_type}")

        return {"success": True, "message": "Commands executed successfully"}
    except Exception as e:
        logger.error(f"Error executing commands: {str(e)}")
        raise HTTPException(
            status_code=500, detail=f"Error executing commands: {str(e)}"
        )


# Include the routers in the main app
app.include_router(audio_router)
app.include_router(agent_router)
app.include_router(robot_router)

if __name__ == "__main__":
    logger.info("Starting Robot Command Service")
    uvicorn.run(app, host="0.0.0.0", port=8000)
