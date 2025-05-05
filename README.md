# Voice Control of GhostRobotics's V60 Through mavlink

This project aims to establish rudimentary control over Ghost Robotics' V60 over the voice medium. For more projects detail, see the [poster](/Documents/CHANDRA_Poster.pdf) and [presentation](/Documents/Chandra_Presentation.pdf).

The code found in this repository is mainly for demonstration purposes. It is done in a very particular and inadvisable manner. The code communicate with the robot instance through a mavlink communication persistent daemon, controlled by a server running on FastAPI. This is implemented this way due to the fact that the V60 simulator running on my WSL 2 instance seems to be interfered by the Docker engine, but the robot's ROS 2 backend is provided in a Docker container, so ROS 2 essentially cannot be used on my end. To implement this workflow in a real situation, a ROS 2 package should be written.

# Project Structure

## The `scripts` folder

This contains utility scripts to help work with the repo:

- `dev.py` turn on both the backend and Streamlit frontend.
- `ghost_sim.sh` turn on the Bullet simulator (assuming that it is already installed appropriately).
- `mvlink.sh` turn on the mvlink GUI, to test that the mavlink compatibility layer has been installed appropriately.

## The `src` folder

This is essentially the project's backend. The important file here is `server.py`, which contains the main routines of the application. There are other library code and a `client.py` file which spins up a simple Streamlit frontend to intake audio.

The `data` folder contains prompts that would be fed to the agents, and some synthetic test cases that we ran to determine the effectiveness of the system.

## The `frontend` folder

This specific folder exists to spin up a slightly more elaborate frontend with better control flow than Streamlit. This is done with ViteJS + Typescript + React. This is mostly to demonstrate, and not necessary for the workings of the project, so installation and execution of this specific section is not vital.

## The `Documents` folder

Contain various documents and design diagrams about the project.

# Possible Commands

- Movement: the robot can move forward, backward, left, right, and any combination of horizontal and lateral movement for some amount of time. This will automatically engage action mode 2.
- Rotation: the robot can rotate "left" (counterclockwise) or "right" (clockwise) for some amount of time. If you use degree, the robot will attempt to estimate it, which might be wrong. This will automatically engage action mode 2.
- Sit: enter action mode 0, which make the robot sit down.
- Stand: enter action mode 2, which make the robot stand up and enable walking around.
- Roll over: trigger the roll over mission for the robot.
- Stop: set all the robot's velocity to 0 for some amount of time.