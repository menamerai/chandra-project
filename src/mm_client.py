import gradio as gr
import time
import requests
import os
from dotenv import load_dotenv
from loguru import logger

load_dotenv(".devcontainer/.env")

# Chatbot demo with multimodal input (text, markdown, LaTeX, code blocks, image, audio, & video). Plus shows support for streaming text.


def print_like_dislike(x: gr.LikeData):
    print(x.index, x.value, x.liked)


def transcribe_audio(file_path):
    """
    Send audio file to the transcription endpoint and return the transcription.
    """
    backend_url = os.getenv("BACKEND_LOCATION")
    if not backend_url:
        raise ValueError("BACKEND_LOCATION is not set in the environment variables")

    with open(file_path, "rb") as audio_file:
        files = {"file": ("audio.wav", audio_file, "audio/wav")}
        response = requests.post(f"{backend_url}/command", files=files)

    if response.status_code == 200:
        result = response.json()
        return result.get("text", ""), result.get("command", "")
    else:
        raise Exception(f"Error: Server returned status code {response.status_code}")


def add_message(history, message):
    for x in message["files"]:
        logger.info(f"Processing file: {x}")
        history.append({"role": "user", "content": {"path": x}})
        try:
            transcription, _ = transcribe_audio(x)
            logger.info(f"Transcription result: {transcription}")
            history.append({"role": "user", "content": transcription})
        except Exception as e:
            logger.error(f"Error transcribing audio: {str(e)}")
            history.append({"role": "user", "content": f"Error transcribing audio: {str(e)}"})

    if message["text"] is not None:
        logger.info(f"Adding text message: {message['text']}")
        history.append({"role": "user", "content": message["text"]})
    return history, gr.MultimodalTextbox(value=None, interactive=False)


def bot(history: list):
    user_message = history[-2]["content"] # for some reason, the last message is empty
    logger.info(f"User message received by bot: {user_message}")
    response = f"**That's cool! You said: {user_message}**"
    history.append({"role": "assistant", "content": ""})
    for character in response:
        history[-1]["content"] += character
        time.sleep(0.05)
        yield history


with gr.Blocks() as demo:
    chatbot = gr.Chatbot(elem_id="chatbot", bubble_full_width=False, type="messages")

    chat_input = gr.MultimodalTextbox(
        interactive=True,
        file_count="multiple",
        placeholder="Enter message or upload file...",
        show_label=False,
        sources=["microphone", "upload"],
    )

    chat_msg = chat_input.submit(
        add_message, [chatbot, chat_input], [chatbot, chat_input]
    )
    bot_msg = chat_msg.then(bot, chatbot, chatbot, api_name="bot_response")
    bot_msg.then(lambda: gr.MultimodalTextbox(interactive=True), None, [chat_input])

    chatbot.like(print_like_dislike, None, None, like_user_message=True)

if __name__ == "__main__":
    demo.launch()