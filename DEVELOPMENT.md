# Installation Guide

To run this repo, you need access to Ubuntu 20.04 LTS. I've personally done this with WSL 2 within Windows 11, but a VM should work. Note that using Docker will **not** work. For some reason, using the Docker engine interfere with running the Bullets Simulation, and crashes it on my machine. You can attempt to test this phenomenon, but for now Docker (and thus the devcontainer setup) will not work and will not be tested on my machine.

Within Ubuntu 20.04, you also need to install `uv`. You can read the documentation [here](https://docs.astral.sh/uv/getting-started/installation/) on how to install `uv`. If you wish to not use `uv`, you can simply use the package manager of your choice and use the `requirements.txt` file to install dependencies, though I offer no guarantee that it will work.

Additionally, if you plan on running the Vite frontend as well, you should install NodeJS, preferably through `nvm`. The documentation can be found [here](https://github.com/nvm-sh/nvm). This isn't exactly necessary, as the frontend's only job is to record and trigger jobs in the FastAPI backend. If you don't want the extra hassle, the bare-boned Streamlit frontend can also be ran from the save directory as the backend.

I'll also assume that you have installed the necessary packages to work with the robot, and will not discuss specifics here. If your installation specifics differs from mine, edit the bash scripts within the `scripts` folder to match.

Once everything is in place, simply install the necessary dependencies by navigating to the root of the project and execute:

```bash
uv sync
```

If you chose to install `nvm` and run the Vite frontend, you should now navigate to the `frontend` folder and install depenencies with:

```bash
npm install
```

# Running The Project

Once the project is correctly installed, we can now attempt to spin it up. Generally, we should first turn on the *simulator*, then the *controller*. Assuming that the simulator is already installed, we cna run it by executing:

```bash
bash ./scripts/ghost_sim.sh
```

The simulator will then be fired up. Once the *simulator* is up and running, the *controller* then can be executed and bound to the simulator by running:

```bash
uv run uvicorn src.server:app
```

Then either the Streamlit frontend ot the Vite frontend can be executed. The Streamlit frontend can be started with:

```bash
uv run streamlit run src/client.py
```

The Vite frontend can be executed by navigating to the `frontend` folder and running:

```bash
npm run dev
```