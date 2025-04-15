## System Architecture Overview  

The Chandra Project comprises three core subsystems:  
1. **Audio Interface Module**: Handles voice command capture via push-to-talk mechanisms and omnidirectional microphones.  
2. **Vision Processing Unit**: Utilizes NVIDIA Jetson Orin Nano's 128-core GPU for real-time object detection and open-sourced Visual Language Model (VLM).
3. **Robotic Integration Layer**: Implements ROS 2 Humble middleware for cross-platform command execution.

---

## Hardware Installation  

### Required Components

- NVIDIA Jetson Orin Nano Developer Kit (8GB RAM minimum)  
- Ghost Robotics V60 robot
- 12V/5A DC Power Supply with barrel connector  

## Software Configuration  

### Base System Setup  

Flash the Jetson module with NVIDIA JetPack 6.2, following the [official setup guide by NVIDIA](https://www.jetson-ai-lab.com/initial_setup_jon.html.)

### Install Package Manager

Ensure that the package manager `uv` is installed. Follow [this guide](https://docs.astral.sh/uv/getting-started/installation/) for specifics on installation.

### Install Project

You can either use the devcontainer that is contained within the project for easy installation, or install the project manually. In either cases, you need to be using specifically Ubuntu 20.04 LTS, whether through an actual desktop environment, through a virtual machine, using the devcontainer, or using Window Subsystem Linux (WSL).

To use the devcontainer, [install Docker Desktop](https://www.docker.com/get-started/), then download [VSCode](https://code.visualstudio.com/download), and follow [this guide](https://code.visualstudio.com/docs/devcontainers/containers) to get familiar with devcontainers and their setup.

To install manually, navigate to any folder you wish to store the source code in, then clone the project with:

```shell
git clone https://github.com/menamerai/chandra-project -b v60
```

Then, install the necessary dependencies and source into the project shell:

```shell
uv sync
```

Finally, install the necessary packages by running this command on the root folder of the project:

```shell
bash .devcontainer/configure.sh
```

---

## Core Functionality  

### Basic Voice Commands

These are the commands that the robot can physically execute. This is different from what you can *tell* the robot to do. For example, the robot might be able to execute **forward X**, where it walks forward for X seconds. However, if you simply to tell it to "walk ahead for a little bit" instead, our system will translate it to some reasonable discrete command, like **forward 2**.

- **"Forward/Backward/Left/Right/ForwardLeft/ForwardRight/BackwardLeft/BackwardRight X"**: Triggers appropriate locomotive drive for lateral/horizontal movements for X seconds. The robot can also optionally interpret step to seconds with a computation ratio of 1 step : 0.5 second.
- **Spin Left/Right X**: Triggers a stationary spinning motion for X seconds to the left or right.
- **Beg**: The robot execute a routine of lifting its head, then lowering its head as if begging.
- **Shake**: The robot slowly bend horizontally from left to right, repeating twice to mimic the action of shaking.
- **Stop X**: The robot does nothing for X seconds.
- **Roll Over**: The robot execute the rolling over routine present in the V60.
- **Sit**: The robot switch to sitting mode.
- **Stand**: The robot switch to *walking* mode. This is notable not the *standing* mode, as in order for the robot to execute locomotive commands, the robot needs to switch to *walking* mode. There are no visual differences, just a state change command that might take time, so it is implemented this way for efficiency's shake.

---

## Frequently Asked Questions  

### General Operation  

**Q: How does the push-to-talk mechanism prevent accidental activations?**  
A: The capacitive touch sensor requires a keyword to activate, similar to Amazon Echo's "Alexa" wake word. Within the demo, we have a visual button that you actually need to click to engage, then click to disengage.

**Q: Will voice calibration be necessary to operate the device?**
A: From our testing right now, voice calibration is not necessary for optimal system operation, as OpenAI's Whisper work on arbitrary voices. However, the default language is English, so if you're mixing languages, that will be difficult to transcribe.

---

For technical support, contact:
**Phan Anh Duong**
Email: duongap@mail.uc.edu

*Documentation version 2.0.0, last updated 2025-04-14*
