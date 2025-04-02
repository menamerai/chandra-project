from enum import Enum
import math
import sys

import rclpy
from geometry_msgs.msg import Twist, Pose
from rclpy.node import Node
from champ_msgs.msg import Pose as PoseLite

from loguru import logger


class Direction(Enum):
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"
    # Extended directions for Cheetah
    UP = "up"  # Body up
    DOWN = "down"  # Body down
    ROLL_LEFT = "roll_left"
    ROLL_RIGHT = "roll_right"
    PITCH_UP = "pitch_up"
    PITCH_DOWN = "pitch_down"
    YAW_LEFT = "yaw_left"
    YAW_RIGHT = "yaw_right"

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
            Twist, "/cmd_vel", 10  # Standard topic for TurtleBot3 velocity commands
        )
        self.timer = None

    def publish_velocity(self, direction: Direction):
        msg = Twist()
        if direction == Direction.FORWARD:
            msg.linear.x = 0.2
            msg.angular.z = 0.0
        elif direction == Direction.BACKWARD:
            msg.linear.x = -0.2
            msg.angular.z = 0.0
        elif direction == Direction.LEFT:
            msg.linear.x = 0.0
            msg.angular.z = 0.5
        elif direction == Direction.RIGHT:
            msg.linear.x = 0.0
            msg.angular.z = -0.5
        self.publisher.publish(msg)

    def reset_velocity(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * sr - sy * sp * cr
    q[1] = sy * cp * sr + cy * sp * cr
    q[2] = sy * cp * cr - cy * sp * sr
    q[3] = cy * cp * cr + sy * sp * sr

    return q


class CheetahVelocityPublisher(Node):
    def __init__(self):
        super().__init__("cheetah_publisher")
        logger.debug("Initializing CheetahVelocityPublisher")
        
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        logger.debug(f"Created velocity publisher on topic 'cmd_vel'")
        
        self.pose_lite_publisher = self.create_publisher(PoseLite, 'body_pose/raw', 10)
        logger.debug(f"Created pose_lite publisher on topic 'body_pose/raw'")
        
        self.pose_publisher = self.create_publisher(Pose, 'body_pose', 10)
        logger.debug(f"Created pose publisher on topic 'body_pose'")
        
        # Default movement parameters
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.body_z_offset = 0.0
        self.roll_value = 0.0
        self.pitch_value = 0.0
        self.yaw_value = 0.0
        logger.debug(f"Movement parameters initialized: linear={self.linear_speed}, "
                     f"angular={self.angular_speed}")

    def publish_velocity(self, direction: Direction):
        logger.info(f"Publishing Cheetah command for direction: {direction.name}")
        
        # Create velocity message
        twist = Twist()
        
        # Create pose messages
        body_pose_lite = PoseLite()
        body_pose_lite.x = 0.0
        body_pose_lite.y = 0.0
        body_pose_lite.z = self.body_z_offset
        body_pose_lite.roll = self.roll_value
        body_pose_lite.pitch = self.pitch_value
        body_pose_lite.yaw = self.yaw_value
        
        # Set values based on direction
        if direction == Direction.FORWARD:
            twist.linear.x = self.linear_speed
            logger.debug(f"Setting linear.x = {self.linear_speed} (FORWARD)")
        elif direction == Direction.BACKWARD:
            twist.linear.x = -self.linear_speed
            logger.debug(f"Setting linear.x = {-self.linear_speed} (BACKWARD)")
        elif direction == Direction.LEFT:
            twist.angular.z = self.angular_speed
            logger.debug(f"Setting angular.z = {self.angular_speed} (LEFT)")
        elif direction == Direction.RIGHT:
            twist.angular.z = -self.angular_speed
            logger.debug(f"Setting angular.z = {-self.angular_speed} (RIGHT)")
        elif direction == Direction.UP:
            body_pose_lite.z = 0.5
            logger.debug(f"Setting body_pose.z = 0.5 (UP)")
        elif direction == Direction.DOWN:
            body_pose_lite.z = -0.5
            logger.debug(f"Setting body_pose.z = -0.5 (DOWN)")
        elif direction == Direction.ROLL_LEFT:
            body_pose_lite.roll = 0.349066  # ~20 degrees
            logger.debug(f"Setting body_pose.roll = 0.349066 (ROLL_LEFT)")
        elif direction == Direction.ROLL_RIGHT:
            body_pose_lite.roll = -0.349066  # ~-20 degrees
            logger.debug(f"Setting body_pose.roll = -0.349066 (ROLL_RIGHT)")
        elif direction == Direction.PITCH_UP:
            body_pose_lite.pitch = 0.174533  # ~10 degrees
            logger.debug(f"Setting body_pose.pitch = 0.174533 (PITCH_UP)")
        elif direction == Direction.PITCH_DOWN:
            body_pose_lite.pitch = -0.174533  # ~-10 degrees
            logger.debug(f"Setting body_pose.pitch = -0.174533 (PITCH_DOWN)")
        elif direction == Direction.YAW_LEFT:
            body_pose_lite.yaw = 0.436332  # ~25 degrees
            logger.debug(f"Setting body_pose.yaw = 0.436332 (YAW_LEFT)")
        elif direction == Direction.YAW_RIGHT:
            body_pose_lite.yaw = -0.436332  # ~-25 degrees
            logger.debug(f"Setting body_pose.yaw = -0.436332 (YAW_RIGHT)")
            
        # Publish velocity command
        logger.debug(f"Publishing twist message: linear.x={twist.linear.x}, angular.z={twist.angular.z}")
        self.velocity_publisher.publish(twist)
        
        # Publish pose commands
        logger.debug(f"Publishing PoseLite message: z={body_pose_lite.z}, roll={body_pose_lite.roll}, "
                     f"pitch={body_pose_lite.pitch}, yaw={body_pose_lite.yaw}")
        self.pose_lite_publisher.publish(body_pose_lite)
        
        # Convert to standard Pose message
        body_pose = Pose()
        body_pose.position.z = body_pose_lite.z
        
        quaternion = quaternion_from_euler(
            body_pose_lite.roll, 
            body_pose_lite.pitch, 
            body_pose_lite.yaw
        )
        
        body_pose.orientation.x = quaternion[0]
        body_pose.orientation.y = quaternion[1]
        body_pose.orientation.z = quaternion[2]
        body_pose.orientation.w = quaternion[3]
        
        logger.debug(f"Publishing Pose message: position.z={body_pose.position.z}, "
                     f"orientation=({body_pose.orientation.x}, {body_pose.orientation.y}, "
                     f"{body_pose.orientation.z}, {body_pose.orientation.w})")
        self.pose_publisher.publish(body_pose)
        logger.info(f"All Cheetah messages published for direction: {direction.name}")

    def reset_velocity(self):
        logger.info("Resetting Cheetah velocity and pose")
        
        # Reset velocity
        twist = Twist()
        logger.debug("Publishing zero velocity twist message")
        self.velocity_publisher.publish(twist)
        
        # Reset pose
        body_pose_lite = PoseLite()
        body_pose_lite.x = 0.0
        body_pose_lite.y = 0.0
        body_pose_lite.z = 0.0
        body_pose_lite.roll = 0.0
        body_pose_lite.pitch = 0.0
        body_pose_lite.yaw = 0.0
        logger.debug("Publishing reset PoseLite message")
        self.pose_lite_publisher.publish(body_pose_lite)
        
        # Reset standard pose
        body_pose = Pose()
        quaternion = quaternion_from_euler(0.0, 0.0, 0.0)
        body_pose.orientation.x = quaternion[0]
        body_pose.orientation.y = quaternion[1]
        body_pose.orientation.z = quaternion[2]
        body_pose.orientation.w = quaternion[3]
        logger.debug("Publishing reset Pose message")
        self.pose_publisher.publish(body_pose)
        logger.info("Cheetah reset completed")


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
        robot_type: Type of robot ("turtlebot" or "cheetah")
    """
    logger.info(f"Starting velocity_pub for {robot_type} robot")
    
    try:
        rclpy.init(args=args)
        logger.debug(f"ROS initialized successfully")
        
        # Select the appropriate node based on robot type
        if robot_type.lower() == "cheetah":
            logger.debug("Creating CheetahVelocityPublisher node")
            node = CheetahVelocityPublisher()
        else:
            logger.debug("Creating VelocityPublisher node")
            node = VelocityPublisher()
            
        # Convert string direction to enum if needed
        if isinstance(direction, str):
            logger.debug(f"Converting string direction '{direction}' to enum")
            direction = Direction.from_string(direction)
        
        logger.info(f"Publishing {direction.name} command to {robot_type} for {execution_time} seconds")
        
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
        logger.info(f"Command execution completed for {robot_type}")
        
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


if __name__ == "__main__":
    # change logger to rebug
    logger.remove()
    logger.add(sys.stderr, level="DEBUG")
    # Use the unified function that supports both robot types
    velocity_pub(direction="forward", execution_time=10, robot_type="cheetah")
    
    # To test with the cheetah robot, uncomment this line:
    # cheetah_velocity_pub(direction="forward", execution_time=3, robot_type="cheetah")
