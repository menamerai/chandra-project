from enum import Enum
import sys
import threading
import subprocess

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from loguru import logger

LINEAR_VELOCITY = 2.0
ANGULAR_VELOCITY = 1.0

class Direction(Enum):
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"
    FORWARD_LEFT = "forward_left"
    FORWARD_RIGHT = "forward_right"
    BACKWARD_LEFT = "backward_left"
    BACKWARD_RIGHT = "backward_right"
    STOP = "stop"

    @staticmethod
    def from_string(direction_str: str):
        try:
            return Direction[direction_str.upper()]
        except KeyError:
            raise ValueError(f"Unknown direction: {direction_str}")


class VelocityPublisher(Node):
    def __init__(self):
        super().__init__("velocity_publisher")
        self.publisher = self.create_publisher(
            Twist, "/cmd_vel", 10  # Standard topic for command
        )
        self.timer = None

    def publish_velocity(self, direction: Direction):
        msg = Twist()
        match direction:
            case Direction.FORWARD:
                msg.linear.x = LINEAR_VELOCITY
                msg.angular.z = 0.0
            case Direction.BACKWARD:
                msg.linear.x = -LINEAR_VELOCITY
                msg.angular.z = 0.0
            case Direction.LEFT:
                msg.linear.x = 0.0
                msg.angular.z = ANGULAR_VELOCITY
            case Direction.RIGHT:
                msg.linear.x = 0.0
                msg.angular.z = -ANGULAR_VELOCITY
            case Direction.FORWARD_LEFT:
                msg.linear.x = LINEAR_VELOCITY
                msg.angular.z = ANGULAR_VELOCITY
            case Direction.FORWARD_RIGHT:
                msg.linear.x = LINEAR_VELOCITY
                msg.angular.z = -ANGULAR_VELOCITY
            case Direction.BACKWARD_LEFT:
                msg.linear.x = -LINEAR_VELOCITY
                msg.angular.z = ANGULAR_VELOCITY
            case Direction.BACKWARD_RIGHT:
                msg.linear.x = -LINEAR_VELOCITY
                msg.angular.z = -ANGULAR_VELOCITY
            case Direction.STOP:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            case _:
                raise ValueError(f"Unknown direction: {direction}")
        logger.debug(f"Publishing velocity: linear.x={msg.linear.x}, angular.z={msg.angular.z}")
        self.publisher.publish(msg)

    def reset_velocity(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)


def velocity_pub(
    args: list[str] | None = None,
    direction: Direction | str = Direction.FORWARD,
    execution_time: int = 5,
    robot_type: str = "turtlebot"
):
    """
    Publish velocity and pose commands to a robot
    
    Args:
        args: ROS arguments
        direction: Direction to move (can be string or Direction enum)
        execution_time: How long to execute the command in seconds
        robot_type: Type of robot to control
    """
    logger.info(f"Starting velocity_pub for robot")
    
    try:
        rclpy.init(args=args)
        logger.debug(f"ROS initialized successfully")
        node = VelocityPublisher()
        logger.debug(f"Node created successfully")
            
        # Convert string direction to enum if needed
        if isinstance(direction, str):
            logger.debug(f"Converting string direction '{direction}' to enum")
            direction = Direction.from_string(direction)
        
        logger.info(f"Publishing {direction.name} command to robot for {execution_time} seconds")
        
        # Publish the command
        node.publish_velocity(direction)
        
        # Wait for the specified execution time
        start_time = node.get_clock().now()
        logger.debug(f"Command published, waiting for {execution_time} seconds")
        
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1e-3)
            current_time = node.get_clock().now()
            elapsed_ns = (current_time - start_time).nanoseconds
            
            if elapsed_ns > execution_time * 1e9:
                logger.debug(f"Execution time reached, resetting velocity")
                node.reset_velocity()
                break
                
        # Clean up
        logger.debug("Cleaning up ROS node")
        node.destroy_node()
        rclpy.shutdown()
        logger.info("ROS shutdown successfully")
        
    except Exception as e:
        logger.error(f"Error in velocity_pub: {str(e)}")
        logger.exception(e)
        if 'node' in locals():
            try:
                node.destroy_node()
            except:
                pass
        try:
            rclpy.shutdown()
        except:
            pass
        raise


def execute_dance_routine():
    """
    Execute the Mini Pupper dance routine
    
    Args:
        args: ROS arguments
    """
    logger.info("Executing Mini Pupper dance routine")
    
    try:
        # Use subprocess to launch the dance routine
        # This avoids the main thread requirement of LaunchService
        # this feels like a hack but it works
        logger.info("Launching dance routine via subprocess")
        process = subprocess.Popen(
            ["ros2", "launch", "mini_pupper_dance", "dance.launch.py"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        # Return immediately but log some info about the process
        logger.info(f"Dance routine subprocess started with PID {process.pid}")
        
        # You could optionally start a monitoring thread if needed
        def monitor_process():
            stdout, stderr = process.communicate()
            logger.info(f"Dance routine completed with return code {process.returncode}")
            if process.returncode != 0:
                logger.error(f"Dance routine stderr: {stderr}")
        
        monitor_thread = threading.Thread(target=monitor_process)
        monitor_thread.daemon = True
        monitor_thread.start()
        
        return {"status": "launched", "pid": process.pid}
        
    except Exception as e:
        logger.error(f"Error launching dance routine: {str(e)}")
        logger.exception(e)
        return {"status": "error", "message": str(e)}


if __name__ == "__main__":
    # change logger to debug
    logger.remove()
    logger.add(sys.stderr, level="DEBUG")
    velocity_pub(direction="forward", execution_time=10)
