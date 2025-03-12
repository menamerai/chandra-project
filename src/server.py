import os
import tempfile

import uvicorn
import whisper
from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import RedirectResponse
from loguru import logger

from src.brigde import Direction, velocity_pub

app = FastAPI(title="Audio Transcription Service")

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


@app.post("/velocity")
async def publish_velocity(direction: Direction, execution_time: int):
    """
    Publishes a constant velocity command to the robot in the specified direction for a given execution time.
    """
    try:
        velocity_pub(direction=direction, execution_time=execution_time)
        return {"message": "Velocity command published successfully"}
    except Exception as e:
        logger.error(f"Error publishing velocity command: {str(e)}")
        raise HTTPException(
            status_code=500, detail=f"Error publishing velocity command: {str(e)}"
        )


@app.post("/command")
async def transcribe_command(file: UploadFile = File(...)):
    """
    Accepts a .wav audio file and returns its transcription.
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
        logger.info("Audio transcription completed successfully")

        # Clean up temporary file
        os.unlink(temp_path)

        # Return the transcribed text
        return {"text": result["text"]}

    except Exception as e:
        # Make sure to clean up if there's an error
        if "temp_path" in locals() and os.path.exists(temp_path):
            os.unlink(temp_path)
        logger.error(f"Error processing audio: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing audio: {str(e)}")


if __name__ == "__main__":
    logger.info("Starting Audio Transcription Service")
    uvicorn.run(app, host="0.0.0.0", port=8000)
