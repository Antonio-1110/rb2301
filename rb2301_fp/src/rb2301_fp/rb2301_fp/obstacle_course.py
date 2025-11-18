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
                '/vrpn_mocap/bingda_007/pose',
                self.optitrack_callback, 
                qos_profile
                )
            
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 5) # Publish to cmd_vel node queue size decreased to 5 to get newer messages  
        self.timer = self.create_timer(0.02, self.timer_callback)  # Changed to 0.02 to get faster update about surrounding

        self.pose = self.last_scan = None
        self.obstacle = self.has_left = self.has_right = self.has_back = self.has_front = self.know = False # avalibility in each direction and know is to prevent the robot from keep making decision when every direction is avalible and obstacle is to record whether there is an obstacle appeared before during the dynamic obstacle part
        self.declare_parameter('turning', False)
        self.move = "start" # keeps track of current move
        self.f_threshold = 0.1525 # distance threshold to deem there is obstacle in front
        self.b_threshold = 0.23 # distance threshold to deem there is obstacle in back
        self.l_r_threshold = 0.177 # distance threshold to deem there is obstacle in left or right
        self.too_close_offset  = 0.06 # push off from the wall when the robot is getting too close to the wall
        self.f_range = (37,323) # angle range in front
        self.b_range = (155,205) # angle range in back
        self.r_range = (230,288) # angle range in right
        self.l_range = (71,130) # angle range in left
        self.y_vel = self.x_vel = 0 # initialize velocity variables

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
    
    def keep_straight(self): # keep the robot facing forward
        if abs(self.pose[2]) > 2: # when the heading deviates more than 2 degrees from 0
            return -self.pose[2]*0.7 # return the reverse of the deviation to correct it, with a coefficient of 0.7
        return 0 # if no deviation then returns zero
    
    def obstacle_angle(self, data=list): # not used
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
    
    def update_data(self): # update the cal_y, cal_x list, and the avaliability of each direction and whether they are too close to the wall
        self.cal_y = [] # list storing the y distance in each angle
        self.cal_x = [] # list storing the x distance in each angle
        for i, value in enumerate(self.last_scan):
            self.cal_x.append(abs(np.cos(np.deg2rad(i+self.pose[2]))*value))
            self.cal_y.append(abs(np.sin(np.deg2rad(i+self.pose[2]))*value)) # becomes wierd when heading is not zero
        # Front
        if min(self.cal_x[0:self.f_range[0]]) < self.f_threshold or min(self.cal_x[self.f_range[1]:]) < self.f_threshold: # check front right and front left
            self.front = False # if it is less than threshold then is not avaliable
            if min(self.cal_x[0:self.f_range[0]]) < self.f_threshold - self.too_close_offset or min(self.cal_x[self.f_range[1]:]) < self.f_threshold - self.too_close_offset:
                self.f_too_close = True # check if it is too close
            else: self.f_too_close = False
        else: self.front, self.f_too_close = True, False
        # Back
        if min(self.cal_x[self.b_range[0]:self.b_range[1]]) < self.b_threshold and min(self.cal_y[self.b_range[0]:self.b_range[1]]) < self.l_r_threshold:
                self.back = False
                if min(self.cal_x[self.b_range[0]:self.b_range[1]]) < self.b_threshold - self.too_close_offset:
                    self.b_too_close = True
                else: self.b_too_close = False
        else: self.back, self.b_too_close = True, False
        # Left
        if min(self.cal_y[self.l_range[0]:self.l_range[1]]) < self.l_r_threshold:
            self.left = False
            if min(self.cal_y[self.l_range[0]:self.l_range[1]]) < self.l_r_threshold - self.too_close_offset:
                self.l_too_close = True
            else: self.l_too_close = False
        else: self.left, self.l_too_close = True, False
        # Right
        if min(self.cal_y[self.r_range[0]:self.r_range[1]]) < self.l_r_threshold:
            self.right = False
            if min(self.cal_y[self.r_range[0]:self.r_range[1]]) < self.l_r_threshold - self.too_close_offset:
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
        if self.last_scan is None:
            return # Does not run if no lidar data received from lidar
        elif np.linalg.norm(self.pose[:2] - self.goal_coordinates) < 0.05: # If distance to goal is less than 0.05m, consider goal reached and exit
            self.get_logger().info("Goal reached! Exiting script")
            raise SystemExit
        
        ###### INSERT CODE HERE ######
        self.update_data()
        self.get_logger().info(f"Pose: {self.pose}")
        if self.get_parameter('turning').value: # was going to do heading but could not make it happen
            self.move_2D(0,0,3)
        else:
            if self.move == "start": # the robot will move to the rightmost position when the node is initialized
                print("starting")
                if -2 < self.pose[2] < 2: # when the robot is not facing zero then it will adjust it first or else the cal_x and cal_y would be affected and messing up the avaliability of each direction
                    if self.right: self.going_r # when the robot is facing zero then move to the right
                    else: self.move = "front" # once moved to the rightmost position then move forward i.e. front
                else: pass # pass because keep_straight() is always called
            elif self.pose[0] > 3.126 and not self.is_simulation: # when the robot is not in simulation it will stop when the x coordinates passed a certain value(hard coded so there is limitation)
                if self.front:self.going_f() # if there is still space in front move forward
                else: # else stop and exit
                    self.move_2D(0,0,0)
                    self.get_logger().info("Goal reached! Exiting script")
                    raise SystemExit
            elif self.pose[0] > 2.16 and not self.is_simulation: # when the robot reaches dynamix obstacle stage, again limitation exist because the coordinates are hard coded
                if self.right: self.y_vel = -0.15 # move to the rightmost position to have maximum time to pass the obstacle before it swings back
                else: 
                    self.y_vel = 0 # stop moving right if already at the rightmost position
                    if not self.front and not self.obstacle: # if there is obstacle in front and obstacle has not appear yet
                        self.obstacle = True # record obstacle has appeared
                        self.last_pose = self.pose[0] # record current x coordinates
                    if self.obstacle: # when the obstacle has appeared and disappeared again meaning that the obstacle has just leave the FOV of the robot
                        if self.pose[0] < self.last_pose+0.52: # move forward 0.52 
                            if self.front: self.x_vel = 0.3 # with maximum speed
                            else: self.x_vel = 0 # when there is not obstacle in front
                        else: 
                            self.obstacle = False # once in position for the second dynamic obstacle then set the obstacle has appeared variable to False again
                    else:
                        self.x_vel = 0 # wait until obstacle has appeared and disappeared
            else:
                if self.front and self.back and self.right and self.left and self.know: # to stop the robot from constantly making decision when every direction is avaliable
                    pass
                else:
                    self.know = False
                    if self.move == "back": # if the robot is going backwards
                        if self.back and self.left: # if both left and back is avaliable
                            self.know = True
                            if self.has_left: # if there left has been NOT avaliable
                                self.going_l() # robot explore left
                                self.move = "left" # change current move to left
                                self.has_left = False # resets variable
                            else:
                                self.going_b() # if there has not been a left wall then go back
                        elif self.left: # if only left is avalible
                            self.has_left = False # resets variable
                            self.going_l() # goes left
                            self.move = "left"
                        elif self.back: # if only back is avalible 
                            self.has_left = True # mark down that left has not been avaliable
                            self.going_b() # goes back
                        else: # if both back and left are not avalible
                            if self.right: self.going_r() # if right is avaliable then goes straight to right
                            else: self.stop() # else stop and let the if self.move == "right" handle the decision making
                            self.know = True
                            self.move = "right"
                            self.has_left = False
                    elif self.move == "right": # similar logic applied to the rest of the direction
                        if self.right and self.back:
                            self.know = True
                            if self.has_back:
                                self.going_b()
                                self.move = "back"
                                self.has_back = False
                            else:
                                self.going_r()
                        elif self.back:
                            self.has_back = False
                            self.going_b()
                            self.move = "back"
                        elif self.right: 
                            self.has_back = True
                            self.going_r()
                        else:
                            if self.front: self.going_f()
                            else: self.stop()
                            self.know = True
                            self.move = "front"
                            self.has_back = False
                    elif self.move == "front":
                        if self.front and self.right:
                            self.know = True
                            if self.has_right:
                                self.going_r()
                                self.move = "right"
                                self.has_right = False
                            else:
                                self.going_f()
                        elif self.right:
                            self.has_right = False
                            self.going_r()
                            self.move = "right"
                        elif self.front: 
                            self.has_right = True
                            self.going_f()
                        else:
                            if self.left:self.going_l()
                            else: self.stop()
                            self.know = True
                            self.move = "left"
                            self.has_right = False
                    elif self.move == "left":
                        if self.left and self.front:
                            self.know = True
                            if self.has_front:
                                self.going_f()
                                self.move = "front"
                                self.has_front = False
                            else:
                                self.going_l()
                        elif self.front:
                            self.has_front = False
                            self.going_f()
                            self.move = "front"
                        elif self.left: 
                            self.has_front = True
                            self.going_l()
                        else:
                            if self.back: self.going_b()
                            else: self.stop()
                            self.know = True
                            self.move = "back"
                            self.has_front = False
                            
        print(
            self.front,
            self.back,
            self.left,
            self.right,
            self.move
        )
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
