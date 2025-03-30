import subprocess


def run_scripts():
    # ros2 launch turtlebot4_ignition_bringup ignition.launch.py
    simulator = subprocess.Popen(
        ["ros2", "launch", "champ_config", "gazebo.launch.py"]
    )
    try:
        simulator.wait()
    except KeyboardInterrupt:
        simulator.terminate()


if __name__ == "__main__":
    run_scripts()
