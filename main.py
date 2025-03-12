import os
import tempfile

import streamlit as st
import whisper

# Set page title
st.title("Audio Transcription App")


# Load Whisper model
@st.cache_resource
def load_whisper_model():
    # Load a small model for faster inference
    model = whisper.load_model("large-v3-turbo")
    return model


model = load_whisper_model()

# Audio input
st.write("Record your audio:")
audio_file = st.audio_input("Record your voice")

# Process the audio file when it's recorded
if audio_file is not None:
    # Display a status message
    with st.spinner("Transcribing audio..."):
        # Create a temporary file to save the audio
        with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as tmp_file:
            tmp_file.write(audio_file.getvalue())
            tmp_filename = tmp_file.name

        # Transcribe the audio
        result = model.transcribe(tmp_filename)
        transcription = result["text"]

        # Clean up the temporary file
        os.unlink(tmp_filename)

    # Display the transcription
    st.subheader("Transcription:")
    st.write(transcription)
