import os
from parser import command_parsing

import requests
import streamlit as st
from dotenv import load_dotenv

from brigde import Direction

# Load environment variables
load_dotenv(".devcontainer/.env")

# Set page title
st.title("Audio Transcription App")

# Input mode select
# input_mode = st.radio("Select input mode:", ("Audio", "Text"))

# Audio input mode
# if input_mode == "Audio":
st.write("Record your audio:")
audio_file = st.audio_input("Record your voice")

# Process the audio file when it's recorded
if audio_file is not None:
    try:
        # Step 1: Send the file to the /command endpoint with its own spinner
        with st.spinner("Sending audio to server for transcription..."):
            files = {"file": ("audio.wav", audio_file.getvalue(), "audio/wav")}
            response = requests.post(
                f'{os.getenv("BACKEND_LOCATION")}/audio/command', files=files
            )

        if response.status_code == 200:
            # Extract the transcription from the response
            result = response.json()
            transcription = result.get("text", "No transcription returned")
            st.subheader("Transcription:")
            st.write(transcription)

            # Step 2: Send the transcription to the /agent endpoint with its own spinner
            with st.spinner("Sending transcription to agent for processing..."):
                agent_response = requests.post(
                    f'{os.getenv("BACKEND_LOCATION")}/agent/agent_run',
                    json={"text": transcription}
                )

            if agent_response.status_code == 200:
                agent_result = agent_response.json()
                st.subheader("Agentic Process Result:")
                st.write(agent_result)
            else:
                st.error(f"Error: Agent endpoint returned status code {agent_response.status_code}")
        else:
            st.error(f"Error: Command endpoint returned status code {response.status_code}")
    except requests.exceptions.ConnectionError:
        st.error("Error: Could not connect to the server")
    except Exception as e:
        st.error(f"Error: {str(e)}")

# Text input mode
# elif input_mode == "Text":
#     st.write("Enter text directly for agent processing:")
#     text_input = st.text_area("Enter your instruction here")

#     if text_input:
#         try:
#             # Ensure the text input is properly formatted
#             formatted_text = text_input.strip()
#             if not formatted_text:
#                 st.error("Error: Text input cannot be empty or whitespace.")
#             else:
#                 with st.spinner("Sending instruction to agent for processing..."):
#                     agent_response = requests.post(
#                         f'{os.getenv("BACKEND_LOCATION")}/agent/agent_run',
#                         json={"text": formatted_text}  # Send properly formatted text
#                     )
#                 if agent_response.status_code == 200:
#                     result = agent_response.json()
#                     st.subheader("Agentic Process Result:")
#                     st.write(result)
#                 else:
#                     st.error(f"Error: Agent endpoint returned status code {agent_response.status_code}")
#         except Exception as e:
#             st.error(f"Error: {str(e)}")
