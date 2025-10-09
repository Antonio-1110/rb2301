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
        self.last = 0.0

    def move_2D(self, x:float=0.0, y:float=0.0, turn:float=0.0):
        twist_msg = Twist()
        x = np.clip(x, -max_translate_velocity, max_translate_velocity)
        y = np.clip(y, -max_translate_velocity, max_translate_velocity)
        turn = np.clip(turn, -max_translate_velocity*2, max_translate_velocity*2)
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.pub_cmd_vel.publish(twist_msg)

    def sub_scan_callback(self, msg):
        """Scan subscriber"""
        self.last_scan = np.array(msg.ranges) # Slices the 721 scan array to return only 36 scans. Feel free to edit

    def timer_callback(self):
        """Controller loop"""

        if self.last_scan is None:
            return # Does not run if the laser message is not received.
        
        ######################## MODIFY CODE HERE ########################
        
        # l_front = self.last_scan[:61] #0-30
        # r_front = self.last_scan[660:] #330-360
        # l_diag = self.last_scan[60:121] #30-60
        # r_diag = self.last_scan[600:661] #300-330
        # u_left = self.last_scan[120:181] #60-90
        # u_right = self.last_scan[540:601] #270-300
        # d_left = self.last_scan[180:241] #90-120
        # d_right = self.last_scan[480:541] #240-270

        # self.last_scan[start_degree*2:end_degree*2+1]

        self.get_logger().debug(str(min(self.last_scan[0:61])) + " / "+ str(min(self.last_scan[660:])))
        #detect front left or front right position of the obstacle -- 1 // cant work cuz the robot will move when avoiding
        #record the movement and react accordingly -- 2
        if min(self.last_scan[0:55]) < 0.36 or min(self.last_scan[666:]) < 0.36:
            if min(self.last_scan[120:231]) < 0.35:
                self.move_2D(0.0, -0.25, 0.0)
                self.last = -0.25
            elif min(self.last_scan[490:600]) < 0.35:
                self.move_2D(0.0, 0.25, 0.0)
                self.last = 0.25
            else: # sufficient space in both left and right
                if self.last != 0.0:  # wouldn't make another decision if the robot is already not moving forward and not runnning into stuff
                    pass
                else: # decide which way to go // random? // base on the approximate direction of the obstacle ahead?
                    if min(self.last_scan[70:111]) > min(self.last_scan[610:651]):
                        self.move_2D(0.0,0.25,0.0)
                        self.last = 0.25
                    else: 
                        self.move_2D(0.0,-0.25,0.0)
                        self.last = -0.25
            # elif min(self.last_scan[130:231]) > min(self.last_scan[490:591]):
            #     self.move_2D(0.0,0.25,0.0)
            #     self.forw = False
            # else:
            #     self.get_logger().debug("no choice")
            #     self.move_2D(0.0,0.-0.25,0.0)
            #     self.forw = False

        else:
            self.move_2D(0.3, 0.0, 0.0)
            self.last = 0.0

        ######################## MODIFY CODE HERE ########################


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()