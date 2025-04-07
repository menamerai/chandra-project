import os
import tempfile
import sys
import timeit

# add src to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))

from parser import command_parsing

import uvicorn
import json
import asyncio
from faster_whisper import WhisperModel, BatchedInferencePipeline
from fastapi import Body, FastAPI, File, HTTPException, UploadFile, APIRouter
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import RedirectResponse
from loguru import logger
from agents import agentic_process
from brigde import Direction, execute_dance_routine, chain_velocity_pub
from dotenv import load_dotenv

dotenv_path = "/chandra-project/.devcontainer/.env"
load_dotenv(dotenv_path=dotenv_path)
os.environ['GEMINI_API_KEY'] = os.getenv("GEMINI_API_KEY")
app = FastAPI(title="Robot Command Service")

# Create a router for Mini Pupper endpoints
audio_router = APIRouter(prefix="/audio", tags=["Audio"])
agent_router = APIRouter(prefix="/agent", tags=["Agent"])
robot_router = APIRouter(prefix="/robot", tags=["Robot"])
minipupper_router = APIRouter(prefix="/minipupper", tags=["Mini Pupper"])

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
    global model, model_name, process, tools, agent
    model_name = "distil-large-v3"
    # model_name = "tiny"
    try:
        # Try GPU first
        # model = WhisperModel(model_name, device="cuda", compute_type="float16")
        model = BatchedInferencePipeline(WhisperModel(model_name, device="cuda", compute_type="float16"))
        logger.info("Whisper model loaded successfully on GPU")
    except Exception as e:
        logger.warning(f"GPU loading failed, falling back to CPU: {str(e)}")
        try:
            # model = WhisperModel(model_name, device="cpu", compute_type="int8")
            model = BatchedInferencePipeline(WhisperModel(model_name, device="cpu", compute_type="int8"))
            logger.info("Whisper model loaded successfully on CPU")
        except Exception as e:
            logger.error(f"Error loading Whisper model: {str(e)}")
            # Continue anyway, will try to load again when needed

    # Initialize the agent
    # process = Agents()
    # process.setup_environment()
    # logger.info("Agent environment set up successfully")
    # tools = [
    #     call_velocity,
    # ]
    # agent = process.agent(model_id="gemini/gemini-2.0-flash", tools=tools)
    # logger.info("Agent initialized successfully")


@robot_router.post("/velocity", deprecated=False)
async def publish_velocity_legacy(
    commands: list[tuple[Direction | str, float]] = Body(
        [
            (Direction.FORWARD, 5.0),
            (Direction.BACKWARD, 5.0),
            (Direction.LEFT, 5.0),
            (Direction.RIGHT, 5.0),
        ]
    ),
):
    """
    Publish velocity command to the robot.
    """
    try:
        chain_velocity_pub(args=None, params=commands)
        return {"message": "Velocity command published successfully (using turtlebot)"}
    except Exception as e:
        logger.error(f"Error publishing velocity command: {str(e)}")
        raise HTTPException(
            status_code=500, detail=f"Error publishing velocity command: {str(e)}"
        )

@audio_router.post("/command", deprecated=False)
async def transcribe_command_legacy(file: UploadFile = File(...)):
    logger.info("Received request to transcribe audio file")
    return await transcribe_audio_file(file)


# Helper function for audio transcription
async def transcribe_audio_file(file: UploadFile):
    logger.info(f"Starting transcription for file: {file.filename}")
    if not file.filename.lower().endswith(".wav"):
        logger.warning(f"Rejected file with unsupported format: {file.filename}")
        raise HTTPException(status_code=400, detail="Only .wav files are supported")

    try:
        start_time = timeit.default_timer()
        logger.info(f"Received file: {file.filename}")
        # Create a temporary file to save the uploaded audio
        with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as temp_file:
            temp_path = temp_file.name
            # Write uploaded file content to temporary file
            temp_file.write(await file.read())

        logger.debug(f"Audio saved to temporary file: {temp_path}")

        # Ensure model is available
        if "model" not in globals():
            global model
            try:
                logger.info("Attempting to load Whisper model on GPU")
                # model = WhisperModel(model_name, device="cuda", compute_type="float16")
                model = BatchedInferencePipeline(WhisperModel(model_name, device="cuda", compute_type="float16"))
            except Exception as e:
                logger.warning(f"GPU loading failed, falling back to CPU: {str(e)}")
                # model = WhisperModel(model_name, device="cpu", compute_type="int8")
                model = BatchedInferencePipeline(WhisperModel(model_name, device="cpu", compute_type="int8"))

        # Transcribe the audio
        segments, info = model.transcribe(temp_path, beam_size=5, language="en", task="transcribe", vad_filter=True, condition_on_previous_text=False, batch_size=16)
        
        # Collect all segment texts
        segments_list = list(segments)  # Convert generator to list
        
        # Join all segment texts to form the complete transcription
        text = " ".join(segment.text for segment in segments_list)
        
        logger.debug(f"Transcribed text: {text}")
        command = command_parsing(text)
        logger.info(f"Transcription completed successfully in {timeit.default_timer() - start_time} seconds")

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
        raise HTTPException(status_code=500, detail=f"Error processing agent request: {str(e)}")


# Add dance endpoint to Mini Pupper router
@minipupper_router.post("/dance")
async def dance_command():
    """
    Make the Mini Pupper robot perform a dance routine.
    """
    try:
        logger.info("Mini Pupper dance command received")
        # Call our separate dance routine function
        result = execute_dance_routine()
        return {"message": f"Dance routine launched successfully, status: {result['status']}"}
    except Exception as e:
        logger.error(f"Error executing dance command: {str(e)}")
        raise HTTPException(
            status_code=500, detail=f"Error executing dance command: {str(e)}"
        )

# Add routers to the main app
app.include_router(audio_router)
app.include_router(agent_router)
app.include_router(robot_router)
app.include_router(minipupper_router)

if __name__ == "__main__":
    logger.info("Starting Robot Command Service")
    uvicorn.run(app, host="0.0.0.0", port=8000)
