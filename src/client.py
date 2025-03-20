import os

import requests
import streamlit as st
from dotenv import load_dotenv
from parser import command_parsing

# Load environment variables
load_dotenv(".devcontainer/.env")

# Set page title
st.title("Audio Transcription App")

# Audio input
st.write("Record your audio:")
audio_file = st.audio_input("Record your voice")

# Process the audio file when it's recorded
if audio_file is not None:
    # Display a status message
    with st.spinner("Sending audio to server for transcription..."):
        # Prepare file for upload
        files = {"file": ("audio.wav", audio_file.getvalue(), "audio/wav")}

        try:
            # Send the file to the FastAPI server
            response = requests.post(
                f'{os.getenv("BACKEND_LOCATION")}/command', files=files
            )

            if response.status_code == 200:
                # Extract the transcription from the response
                result = response.json()
                transcription = result.get("text", "No transcription returned")
                command = command_parsing(transcription) #NOTE: parsing commands

            else:
                transcription = (
                    f"Error: Server returned status code {response.status_code}"
                )
        except requests.exceptions.ConnectionError:
            transcription = "Error: Could not connect to transcription server"
        except Exception as e:
            transcription = f"Error: {str(e)}"

    # Display the transcription
    st.subheader("Transcription:")
    st.write(transcription)
    #TODO: Remove this later, only for testing
    if command: 
        st.write(f"Action: {command['action']}")
        st.write(f"Parameter: {command['parameter']}")
    else: 
        st.write("Command parsing failed")
