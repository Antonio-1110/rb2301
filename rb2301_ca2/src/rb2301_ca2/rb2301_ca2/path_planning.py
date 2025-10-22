import numpy as np
import heapq
import rclpy
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity

from rclpy.qos import (
    ReliabilityPolicy,
    QoSProfile,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from PIL import Image
from geometry_msgs.msg import Twist


np.set_printoptions(
    2, suppress=True, threshold=np.inf
)  # Print numpy arrays to specified d.p., suppress scientific notation (e.g. 1e-5), and do not truncate

set_logger_level("waypoint", level=LoggingSeverity.DEBUG) # Configure to either LoggingSeverity.INFO or LoggingSeverity.DEBUG  

is_simulation = True # Remember to configure this to False if testing for the real lab setup
if is_simulation:
    max_translate_velocity = 1.4
else:
    max_translate_velocity = 0.3 # Please keep this in place; 0.3m/s is more than fast enough 

occupancy_grid_resolution = 0.2

sim_goal_list = [(3.4, -3.6), (3.2, 0.2), (2.4, -3.6), (-0.4, -3.8)]
sim_grid_start = (-1.0, -5.0)

irl_goal_list = [(2.1, -1.7), (2.3, -0.3), (1.5, -1.7), (0.3, -1.7)]
irl_grid_start = (0.1, -1.9)


class WaypointNode(Node):
    '''Node to calculate path and move robot towards given goal_coordinates, using pose info from either gazebo odometer or optitrack'''
    def __init__(self, map_array:np.array, goal_list:list, is_simulation:bool=True):
        super().__init__('waypoint')
        self.get_logger().info("Starting WaypointNode")

        self.is_simulation = is_simulation

        # Subscribe to the dynamic_pose topic from Gazebo that publishes ground-truth pose data
        if self.is_simulation:
            self.subscription = self.create_subscription(Odometry, 'odom', self.odometer_callback, 2)
        else:
            qos_profile = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)

            self.map_sub = self.create_subscription( 
                PoseStamped,
                '/vrpn_mocap/bingda_003/pose',
                self.optitrack_callback, 
                qos_profile
                )
            
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) # Publish to cmd_vel node       
        self.timer = self.create_timer(0.05, self.timer_callback)  # Runs at 20Hz. Can be changed.

        self.goal_list = goal_list
        self.map_array = map_array
        self.wp = []
        self.counter = 0
        self.pose = None
        self.last_x_diff = 0
        self.last_y_diff = 0

    def coordinates_to_index(self, coordinates, robot=False):
        if  robot:
            x = np.clip(round((coordinates[0] + 1.0) / 0.2), 0, 30)
            y = np.clip(round((coordinates[1] + 5.0) / 0.2), 0, 35)
        else:
            x = round((coordinates[0] + 1.0) / 0.2)
            y = round((coordinates[1] + 5.0) / 0.2)
        return (x,y)

    def index_to_coordinates(self, index):
        # x = round(index[0] * 0.2 - 1.0, 2)
        # y = round(index[1] * 0.2 - 5.0, 2)
        x = index[0] * 0.2 - 1.0
        y = index[1] * 0.2 - 5.0
        return (x,y)
    
    def move_to_goal(self):
        coord_location = self.index_to_coordinates(self.wp[0])
        x_diff = self.pose[0] - coord_location[0] - 0.1 # 0.1 offset to make the coordinates in the center of the 0.2*0.2 block
        y_diff = self.pose[1] - coord_location[1] - 0.1 # 0.1 offset to make the coordinates in the center of the 0.2*0.2 block
        if abs(x_diff) < 0.05 and abs(y_diff) < 0.05:
            self.last_x_diff = 0
            self.last_y_diff = 0
            self.wp.pop(0)
            self.move_2D(0,0,0.0)
        else:
            self.last_x_diff+=x_diff # x integrate part
            self.last_y_diff+=y_diff # y integrate part
            total_x_diff = -x_diff - self.last_x_diff
            total_y_diff = -y_diff - self.last_y_diff
            self.move_2D(total_x_diff, total_y_diff, 0.0)
            self.get_logger().debug(f'{total_x_diff}, {total_y_diff}')


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
        x = np.clip(x, -max_translate_velocity, max_translate_velocity)
        y = np.clip(y, -max_translate_velocity, max_translate_velocity)
        turn = np.clip(turn, -max_translate_velocity*2, max_translate_velocity*2)
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.publisher_.publish(twist_msg)

    def set_waypoints(self, waypoints:list):
        '''Set new waypoints when a goal has been reached'''
        self.goal_reached = False
        self.waypoints = waypoints
        self.current_waypoint_idx = 0

    def timer_callback(self):
        """Controller loop. Insert path planning and PID control logic here"""
        if self.pose is None:
            return # Does not run if no pose received from Odom or Optitrack
        self.get_logger().debug(f"Pose: {self.pose}")

        ###### INSERT CODE HERE ######
        if self.wp:
            # self.counter+=1
            self.move_to_goal()
            return # tell the robot how to move based on self.path_to_goal
        else:
            while self.goal_list == []:
                self.get_logger().debug('No more goal to reach')
                return
            self.grid = Grid(self.map_array, self.coordinates_to_index((self.pose[0],self.pose[1]),robot=True), self.coordinates_to_index(self.goal_list.pop(0)))
            self.path_to_goal, self.cost, self.wp = self.grid.a_star()
            self.grid.draw_grid_map(waypoints=self.wp, path=self.path_to_goal)
            # print(self.path_to_goal, self.cost)
        # if self.counter>60:
        #     self.counter = 0
        #     self.know_where_to_go=False
        #     return
        ###### INSERT CODE HERE ######


class Grid():
    '''
    Grid class to use with occupancy grid. Contains the following functions:
        check_grid_validity : Uses flood fill to check if there's a valid path from start to goal position
        draw_grid_map : Creates a colour image of the grid, as well as waypoints and full solution path if given.

    __init__ input Args:
        grid_array : 2D numpy array representing the occupancy grid
        starting_position : tuple of starting indices within the numpy array
        goal_position : tuple of goal indices within the numpy array. Works with negative indices as well
    '''
    def __init__(self, grid_array:np.array=np.array([]), starting_position:tuple=(0,0), goal_position:tuple=(-1,-1)):
        self.grid = grid_array
        self.shape = self.grid.shape
        self.starting_position = starting_position

        # If goal position given with negative index, need to convert to +ve
        if goal_position[0] < 0:
            goal_x = self.shape[0] + goal_position[0]
        else:
            goal_x = goal_position[0]
        if goal_position[1] < 0:
            goal_y = self.shape[1] + goal_position[1]
        else:
            goal_y = goal_position[1]   

        self.goal_position = (goal_x, goal_y)

    def check_grid_validity(self):
        '''Use flood fill to check if there's a viable path between start and goal positions'''
        grid = self.grid.copy()
        flood_stack = [(self.goal_position[0], self.goal_position[1])]
        while flood_stack:
            tile = flood_stack[0]
            del flood_stack[0]
            try:
                next_tile = (tile[0]+1, tile[1])
                if grid[next_tile] == 0:
                    grid[next_tile] = 1
                    flood_stack.append(next_tile)
            except:pass
            try:
                next_tile = (tile[0]-1, tile[1])
                if grid[next_tile] == 0:
                    grid[next_tile] = 1
                    flood_stack.append(next_tile)
            except:pass
            try:
                next_tile = (tile[0], tile[1]+1)
                if grid[next_tile] == 0:
                    grid[next_tile] = 1
                    flood_stack.append(next_tile)
            except:pass
            try:
                next_tile = (tile[0], tile[1]-1)
                if grid[next_tile] == 0:
                    grid[next_tile] = 1
                    flood_stack.append(next_tile)
            except:pass
        if grid[self.starting_position] == 1: # Means the flood is able to reach starting position from the ending position
            return True
        else:
            return False

    def draw_grid_map(self, waypoints:list=(), path:list=(), obstacle_threshold:float=50):
        '''Creates an image of the maze and path taken. Maze walls in blue, empty space in white, path taken in green and waypoints in red

        Args:
            waypoints : list (or other iterable) of tuple coordinates representing all the grid indices for the waypoints. Will be represented in red, takes precedence over path
            path : list (or other iterable) of tuple coordinates representing all the grid indices forming the solution path. Will be represented in green
            obstacle_threshold : Optional float to indicate threshhold for whether a grid is considered occupied. Not important for ca2             
        '''
        image_grid = np.ones((self.grid.shape[0],self.grid.shape[1],3), dtype=np.uint8)
        image_grid[self.grid <= obstacle_threshold] = (255,255,255)
        image_grid[self.grid > obstacle_threshold] = (0,0,255)

        for x, y in path:
            image_grid[x][y] = (0,255,0)

        for point in waypoints:
            image_grid[point] = (255,0,0)

        image_grid = np.flip(image_grid, axis=1)[::-1]
        img = Image.fromarray(image_grid, 'RGB')

        # Resize image
        base_width = 500
        wpercent = (base_width / float(img.size[0]))
        hsize = int((float(img.size[1]) * float(wpercent)))
        img = img.resize((base_width, hsize), Image.Resampling.NEAREST)

        img.show()
    
    def draw_heuristic_map(self):
        h_map = np.zeros((len(self.grid),len(self.grid[0])))
        max = 0 # init value with no reason
        min = 100 # init value with no reason
        for i in range(len(h_map)):
            for j in range(len(h_map[i])):
                h_map[i][j] = np.linalg.norm(np.array((i,j))-np.array(self.goal_position))
                if h_map[i][j] > max:
                    max = h_map[i][j]
                if h_map[i][j] < min:
                    min = h_map[i][j]
        image_grid = np.ones((len(h_map),len(h_map[0]),3), dtype=np.uint8)
        for i in range(len(image_grid)):
            for j in range(len(image_grid[i])):
                image_grid[i][j] = (0,0,(h_map[i][j]-min)/(max-min)*255) #normalizing
        image_grid = np.flip(image_grid, axis=1)[::-1]
        img = Image.fromarray(image_grid, 'RGB')

        # Resize image
        base_width = 500
        wpercent = (base_width / float(img.size[0]))
        hsize = int((float(img.size[1]) * float(wpercent)))
        img = img.resize((base_width, hsize), Image.Resampling.NEAREST)

        img.show()

    def check_neighbors(self, current):
        temp = []
        x = [current[0]-1, current[0]+1]
        y = [current[1]-1, current[1]+1]
        for i in x:
            if self.grid[i][current[1]] < 50 and i > 0 and i < 30:
                temp.append((i,current[1]))
        for j in y:
            if self.grid[current[0]][j] < 50 and j > 0 and j < 35:
                temp.append((current[0],j))
        return temp

        

    def a_star(self):
        # g = manhattan distance? A: Its just 1 regardless cuz we moving by block, all the edges are equal cost
        # h = euclidean distance
        if not self.check_grid_validity():
            print("Cannot reach")
            return [], 0
        path = {self.starting_position:self.starting_position}
        g = {}
        g[self.starting_position] = 0
        f = []
        heapq.heapify(f)
        heapq.heappush(f, (np.linalg.norm(np.array((self.starting_position))-np.array(self.goal_position)), self.starting_position))
        while f:
            current = heapq.heappop(f)[1]
            if current == self.goal_position:
                cost = g[current]
                ans = []
                while current != self.starting_position:
                    ans.insert(0,current)
                    current = path[current]
                ans.insert(0,current)
                waypoints = [self.starting_position]
                for i in range(1, len(ans)-1):
                    if abs(ans[i-1][0] - ans[i][0]) != abs(ans[i+1][0]-ans[i][0]) or abs(ans[i-1][1] - ans[i][1]) != abs(ans[i+1][1]-ans[i][1]):
                        waypoints.append(ans[i])
                waypoints.append(self.goal_position)
                return ans, cost, waypoints
            x = abs(path[current][0] - current[0])
            y = abs(path[current][1] - current[1])
            for i in self.check_neighbors(current):
                if abs(i[0]-current[0]) != x or abs(i[1]-current[1]) != y:
                    diff = 1.3
                else:
                    diff = 1
                if i not in g or g[i] > g[current] + diff:
                    g[i] = g[current] + diff
                    path[i] = current
                    heapq.heappush(f, (g[i]+np.linalg.norm(np.array((i))-np.array(self.goal_position)), i))

            


def main(args=None):
    global is_simulation
    print("Starting path planning")
    rclpy.init(args=args)

    # Load the proper occupancy grid numpy array
    import os
    filepath = os.path.dirname(os.path.realpath(__file__))
    if is_simulation:
        map_array = np.load(filepath + '/ca2_sim_map.npy', allow_pickle=True)
        waypoint = WaypointNode(map_array, sim_goal_list, is_simulation)
    else:
        map_array = np.load(filepath + '/ca2_irl_map.npy', allow_pickle=True)
        waypoint = WaypointNode(map_array, irl_goal_list, is_simulation)

    # Start spinning the waypoint node and only stop once SystemExit error is raised within the node callback
    try:
        rclpy.spin(waypoint)
    except SystemExit:
        print("Shutting down")

    waypoint.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()