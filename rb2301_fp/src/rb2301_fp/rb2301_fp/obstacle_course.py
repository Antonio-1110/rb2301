import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

np.set_printoptions(2, suppress=True, threshold=np.inf) # Print numpy arrays to specified d.p., suppress scientific notation and do not truncate
set_logger_level("obstaclecourse", level=LoggingSeverity.INFO) # Configure to either LoggingSeverity.INFO or LoggingSeverity.DEBUG  

class LidarNode(Node):
    def __init__(self):
        """Node constructor"""
        super().__init__("LidarNode")
        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # Subscribe to lidar
        self.is_simulation = None

    def sub_scan_callback(self, msg):
        """Scan subscriber"""
        if len(msg.ranges) <= 360: 
            self.is_simulation = True
        else:
            self.is_simulation = False

class ObstacleCourseNode(Node):
    '''Node to navigate obstacle course, using pose from either gazebo odometer or optitrack and lidar scan data'''
    def __init__(self, is_simulation:bool=True):
        super().__init__('obstaclecourse')
        self.get_logger().info("Starting ObstacleCourseNode")

        self.is_simulation = is_simulation

        if is_simulation:
            self.max_translate_velocity = 1.4
            self.ref_vel = 0.4
            self.goal_coordinates = np.array((4.8, -2.6))
        else:
            self.max_translate_velocity = 0.3 # Please keep this in place; 0.3m/s is more than fast enough 
            self.goal_coordinates = np.array((5.2, -2.6))
            self.ref_vel = 0.15

        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # Subscribe to LiDAR scan data

        # Subscribe to the dynamic_pose topic from Gazebo that publishes ground-truth pose data
        if self.is_simulation:
            self.subscription = self.create_subscription(Odometry, 'odom', self.odometer_callback, 2)
        else:
            qos_profile = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)

            self.map_sub = self.create_subscription( 
                PoseStamped,
                '/vrpn_mocap/bingda_009/pose',
                self.optitrack_callback, 
                qos_profile
                )
            
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 5) # Publish to cmd_vel node queue size decreased to 5 to get newer messages  
        self.timer = self.create_timer(0.02, self.timer_callback)  # Changed to 0.02 to get faster update about surrounding

        self.pose = None
        self.last_scan = None
        self.obstacle = False
        self.has_left = False
        self.know_direction = True
        self.declare_parameter('turning', False)
        self.last_move = "start"
        self.threshold = 0.165
        self.threshold_offset = 0.08
        self.too_close = 0.114
        self.f_range = (22,338)
        self.b_range = (154,206)
        self.r_range = (231,290)
        self.l_range = (70,129)
        self.y_vel = 0
        self.x_vel = 0

    def sub_scan_callback(self, msg):
        """Scan subscriber"""
        if len(msg.ranges) <= 360:
            self.last_scan = np.array(msg.ranges)
        else:
            self.last_scan = np.array(msg.ranges)[::2] 

    def yaw_from_quaternion(self, q):
        '''Returns yaw angle (in rad) for orientation based on given quaternion input q'''
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def optitrack_callback(self, msg:PoseStamped):
        '''Callback to calculate 2D pose info from Optitrack node. Pose info includes x and y coordinates, as well as heading in degrees.
        This callback will run everytime the rclpy executor spins'''
        x, y = msg.pose.position.x, msg.pose.position.y
        heading = np.rad2deg(self.yaw_from_quaternion(msg.pose.orientation))
        self.pose = np.array((x,y,heading))
        return self.pose

    def odometer_callback(self, msg):
        '''Callback to calculate 2D pose info from Gazebo odomoter. Pose info includes x and y coordinates, as well as heading in degrees.
        This callback will run everytime the rclpy executor spins'''
        latest_pose_msg = msg.pose.pose 
        heading = np.rad2deg(self.yaw_from_quaternion(latest_pose_msg.orientation))
        self.pose = np.array((latest_pose_msg.position.x, latest_pose_msg.position.y, heading))
        return self.pose

    def move_2D(self, x:float=0.0, y:float=0.0, turn:float=0.0):
        '''Publishes a Twist message to ROS to move a robot. Inputs are x and y linear velocities, as well as turn (z-axis yaw) angular velocity.'''
        twist_msg = Twist()
        x = np.clip(x, -self.max_translate_velocity, self.max_translate_velocity)
        y = np.clip(y, -self.max_translate_velocity, self.max_translate_velocity)
        turn = np.clip(turn, -self.max_translate_velocity*2, self.max_translate_velocity*2)
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.publisher_.publish(twist_msg)
    
    def keep_straight(self):
        if abs(self.pose[2]) > 2:
            return -self.pose[2]*0.9
        return 0
    
    def obstacle_angle(self, data=list):
        obstacles = []
        starting = 0
        for i, _ in enumerate(data):
            # print(f'degree = {i}, value = {_}')
            if abs(data[starting] - data[i+1 if i+1 < 360 else 0]) > 0.05: # checking should be more robust than this
                ending = i
                if ending - starting > 6  and 0.08 < abs(data[starting]) < 1:
                    obstacles.append((starting, ending))
                starting = i+1
        if 359 - starting > 6 and 0.08 < abs(data[starting]) < 1:
            obstacles.append((starting, 359))
        return obstacles
    
    def update_data(self):
        self.cal_y = []
        self.cal_x = []
        for i, value in enumerate(self.last_scan):
            self.cal_x.append(abs(np.cos(np.deg2rad(i+self.pose[2]))*value))
            self.cal_y.append(abs(np.sin(np.deg2rad(i+self.pose[2]))*value)) # it becomes wierd when heading is not zero
        # Front
        if min(self.cal_x[0:self.f_range[0]]) < self.threshold and min(self.cal_y[0:self.f_range[0]]) < self.threshold or min(self.cal_x[self.f_range[1]:]) < self.threshold and min(self.cal_y[self.f_range[1]:]) < self.threshold: # check front right and front left
            self.front = False
            if min(self.cal_x[0:self.f_range[0]]) < self.too_close or min(self.cal_x[self.f_range[1]:]) < self.too_close:
                self.f_too_close = True
            else: self.f_too_close = False
        else: self.front, self.f_too_close = True, False
        # Back
        if min(self.cal_x[self.b_range[0]:self.b_range[1]]) < self.threshold + self.threshold_offset and min(self.cal_y[self.b_range[0]:self.b_range[1]]) < self.threshold: # extra offset because lidar is not in the center of the robot
                self.back = False
                if min(self.cal_x[self.b_range[0]:self.b_range[1]]) < self.too_close + self.threshold_offset:
                    self.b_too_close = True
                else: self.b_too_close = False
        else: self.back, self.b_too_close = True, False
        # Left
        if min(self.cal_y[self.l_range[0]:self.l_range[1]]) < self.threshold and min(self.cal_x[self.l_range[0]:self.l_range[1]]) < self.threshold + self.threshold_offset:
            self.left = False
            if min(self.cal_y[self.l_range[0]:self.l_range[1]]) < self.too_close:
                self.l_too_close = True
            else: self.l_too_close = False
        else: self.left, self.l_too_close = True, False
        # Right
        if min(self.cal_y[self.r_range[0]:self.r_range[1]]) < self.threshold and min(self.cal_x[self.r_range[0]:self.r_range[1]]) < self.threshold + self.threshold_offset:
            self.right = False
            if min(self.cal_y[self.r_range[0]:self.r_range[1]]) < self.too_close :
                    self.r_too_close = True
            else: self.r_too_close = False
        else: self.right, self.r_too_close = True, False

    def going_r(self):
        self.y_vel = -self.ref_vel 
        if self.b_too_close:
            self.x_vel = self.ref_vel/3
        else: self.x_vel = 0
    def going_l(self):
        self.y_vel = self.ref_vel
        if self.f_too_close:
            self.x_vel = -self.ref_vel/3
        else: self.x_vel = 0
    def going_f(self):
        self.x_vel = self.ref_vel
        if self.r_too_close:
            self.y_vel = self.ref_vel/3
        else: self.y_vel = 0
    def going_b(self):
        self.x_vel = -self.ref_vel
        if self.l_too_close:
            self.y_vel = -self.ref_vel/3
        else: self.y_vel = 0
    def stop(self):
        self.x_vel = 0
        self.y_vel = 0

    def timer_callback(self):
        """Controller loop. Insert path planning and PID control logic here"""
        if self.pose is None:
            print("No pose detected")
            return # Does not run if no pose received from Odom or Optitrack
            
        elif np.linalg.norm(self.pose[:2] - self.goal_coordinates) < 0.05: # If distance to goal is less than 0.05m, consider goal reached and exit
            self.get_logger().info("Goal reached! Exiting script")
            raise SystemExit
        
        ###### INSERT CODE HERE ######
        if self.last_scan is None:
            return
        self.update_data()
        self.get_logger().info(f"Pose: {self.pose}")
        if self.get_parameter('turning').value:
            self.move_2D(0,0,3)
        else:
            if self.last_move == "start":
                print("starting")
                if self.right: self.going_r()
                else:self.last_move = "right"
            elif self.pose[0] > 3.135 and not self.is_simulation: 
                if self.front:self.going_f()
                else: self.stop()
            elif self.pose[0] > 2.09 and not self.is_simulation:
                if self.right: self.going_r()
                else:
                    if not self.front and not self.obstacle:
                        self.obstacle = True
                        self.last_pose = self.pose[0]
                        self.stop()
                    if self.obstacle:
                        if self.pose[0] < self.last_pose+0.5:
                            if self.front: self.move_2D(0.3,0,self.keep_straight()) 
                            return
                        else: 
                            self.stop()
                            self.obstacle = False
            else:
                if self.front and self.back and self.left and self.right and self.know_direction:
                    print("all true")
                else:
                    self.know_direction = False
                    if self.last_move == "back":
                        if not self.back:
                            if self.right: self.going_r()
                            else:
                                self.stop()
                                self.last_move = "right"
                        else:
                            print(1)
                            self.going_b()
                            self.last_move = "left"
                            self.know_direction = True
                    elif self.last_move == "right":
                        if not self.right:
                            if self.front: self.going_f()
                            else:
                                self.stop()
                                self.last_move = "front"
                        else:
                            self.going_r()
                            self.last_move = "back"
                            self.know_direction = True
                    elif self.last_move == "front":
                        if not self.front:
                            if self.left: self.going_l()
                            else:
                                self.stop()
                                self.last_move = "left"
                        else:
                            self.going_f()
                            self.last_move = "right"
                            self.know_direction = True
                    elif self.last_move == "left":
                        if not self.left:
                            self.has_left = True
                            if self.back:
                                self.going_b()
                                print(2)
                            else:
                                self.stop()
                                self.last_move = "back"
                        elif self.has_left:
                            self.has_left = False
                            self.going_l()
                            self.last_move = "front"
                            self.know_direction = True
                        elif self.back:
                            self.going_b()
                            print(3)
                            self.know_direction = True
                        else:
                            self.going_r()
                            self.last_move = "back"
        self.move_2D(self.x_vel,self.y_vel,self.keep_straight())
        ###### INSERT CODE HERE ######
                

def main(args=None):
    print("Starting obstacle course")
    rclpy.init(args=args)

    # Use lidar scan length to determine if in simulation or not
    lidar = LidarNode()
    while lidar.is_simulation is None:
        rclpy.spin_once(lidar)
    is_simulation = lidar.is_simulation
    lidar.destroy_node()

    # Start spinning the obstacle node and only stop once SystemExit error is raised within the node callback
    obstacle = ObstacleCourseNode(is_simulation)
    try:
        rclpy.spin(obstacle)
    except SystemExit:
        print("Shutting down")
    obstacle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()