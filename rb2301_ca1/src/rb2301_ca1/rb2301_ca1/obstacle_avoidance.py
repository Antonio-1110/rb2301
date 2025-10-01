import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

np.set_printoptions(
    2, suppress=True
)  # Print numpy arrays to specified d.p. and suppress scientific notation (e.g. 1e-5)

max_translate_velocity = 0.4 # Can be implemented as parameter
max_turn_velocity = max_translate_velocity * 2 # Can be implemented as parameter
set_logger_level("obstacle_avoidance", level=LoggingSeverity.DEBUG) # Configure to either LoggingSeverity.INFO or LoggingSeverity.DEBUG  

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        """Node constructor"""
        super().__init__("obstacle_avoidance")
        self.get_logger().info("Starting Obstacle Avoidance")

        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)  # Publish to cmd_vel node
        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # The subscriber to the Lidar ranges.
        self.last_scan = None # Copied laser scan message

        self.timer = self.create_timer(0.05, self.timer_callback)  # Runs at 20Hz. Can be changed.

    def move_2D(self, x: float = 0.0, y: float = 0.0, turn: float = 0.0):
        """Publishes a twist command to move in 2D space. +ve x is forwards, +ve y is left, and +ve turn is anticlockwise"""
        twist_msg = Twist()
        # Please keep the max velocity caps to protect the irl robots
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = (
            min(float(x), max_translate_velocity),
            min(float(y), max_translate_velocity),
            0.0,
        )
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = (
            0.0,
            0.0,
            min(float(turn), max_turn_velocity),
        )
        self.last_x = x
        self.last_y = y
        self.pub_cmd_vel.publish(twist_msg)

    def sub_scan_callback(self, msg):
        """Scan subscriber"""
        self.last_scan = np.array(msg.ranges)[::10] # Slices the 721 scan array to return only 36 scans. Feel free to edit

    def timer_callback(self):
        """Controller loop"""

        if self.last_scan is None:
            return # Does not run if the laser message is not received.
        
        ######################## MODIFY CODE HERE ########################
        # self.get_logger().debug(str(self.last_scan))  
        front = np.concatenate((self.last_scan[65:],self.last_scan[:-64]))
        left = self.last_scan[12:25]
        right = self.last_scan[47:60]
        self.get_logger().debug(str(min(front)))
        if min(front) < 0.3:
            if self.last_x == 0 and self.last_y != 0:
                self.get_logger().debug("1")
                return
            l = min(left)
            r = min(right)
            if l > r and l > 0.2:
                self.move_2D(0.0,0.2,0.0)
            elif r > l and r > 0.2:
                self.move_2D(0.0,-0.2,0.0)
            elif r == l and r > 0.2:
                self.move_2D(0.0,-0.2,0.0)
            else:
                self.get_logger().debug("stuck")
                self.move_2D(0.0,0.0,0.0)
        else:
            self.move_2D(0.2, 0.0, 0.0)



        ######################## MODIFY CODE HERE ########################


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
