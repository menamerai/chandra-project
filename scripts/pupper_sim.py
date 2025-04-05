import subprocess


def run_scripts():
    """
    launch

    ros2 launch mini_pupper_bringup bringup.launch.py hardware_connected:=False

    and 
    
    ros2 launch mini_pupper_bringup rviz.launch.py
    
    in 2 separate processes
    """
    bringup = subprocess.Popen(["ros2", "launch", "mini_pupper_bringup", "bringup.launch.py", "hardware_connected:=False"])
    rviz = subprocess.Popen(["ros2", "launch", "mini_pupper_bringup", "rviz.launch.py"])

    try:
        bringup.wait()
        rviz.wait()
    except KeyboardInterrupt:
        bringup.terminate()
        rviz.terminate()


if __name__ == "__main__":
    run_scripts()
