# The Chandra Project

## Abstract

This project aims to develop a modular attachment for robots to assist visually
impaired individuals by enhancing their ability to navigate and interact with their
surroundings. The attachment will integrate an audio module, a camera module,
and an NVIDIA Jetson to enable the robot to process voice commands, detect
objects in real-time, and provide audible notifications. Designed for universal
compatibility, the module minimizes reliance on proprietary sensors, ensuring
adaptability across various robot platforms. By prioritizing affordability,
accessibility, and open-source development, the project seeks to create a
practical, impactful solution that improves independence and quality of life for
visually impaired users.

## Description

The goal is to create a plug-in module with 1-2 NVIDIA Jetson Orin Nano that would theoretically give robots the ability to listen and execute voice commands. Specifically, this would add a push-to-talk mechanism on some control interface to capture user audio. The recognized commands will be one-word action commands for simplicity. After the command is captured and recognized, the robot will execute a subroutine that correlate with that voice command. This module should be integrated with Brutus (I don't know what the exact make and model of Brutus is) for demonstration purposes.

### Minimum Viable Product

- Robot can capture audio either directly using its integrated mic or using the plug-in's mic using the push-to-talk mechanism.
- Robot can save and send audio to plug-in for processing.
- Plug-in module can use models and data processing techniques (details hashed out later) to translate audio into an entry in a list of pre-defined commands.
- Robot can trigger command subroutine upon receiving it from the module. The MVP commands right now include: sit, stand, bark, circle (the robot does a 360 turn), dance (I remember seeing the robot dance so this might not be terribly hard to integrate), forward X (replaced with any direction and unit of measurement for length).

### Most Lovable Product

- Additional commands that are difficult to implement but would be cool to have are: recognize (use a VLM to verbalize what it is seeing), follow, sequence of directional commands.
- A "talk" mode with TTS to chat with the robot.
