from enum import Enum

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class Direction(Enum):
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"

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


def velocity_pub(
    args: list[str] | None = None,
    direction: Direction | str = Direction.FORWARD,
    execution_time: int = 5,
):
    rclpy.init(args=args)
    node = VelocityPublisher()
    if isinstance(direction, str):
        direction = Direction.from_string(direction)
    node.publish_velocity(direction)
    start_time = node.get_clock().now()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=1e-3)
        if (node.get_clock().now() - start_time).nanoseconds > execution_time * 1e9:
            node.reset_velocity()
            break
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    print("Publishing velocity command...")
    velocity_pub(direction="forward", execution_time=3)
