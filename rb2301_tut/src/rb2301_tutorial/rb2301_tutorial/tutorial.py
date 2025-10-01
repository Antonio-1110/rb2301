# 1. IMPORT
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn

# NODE CLASS
class tutorial(Node):

    # 2. NODE PROPERTIES
    pose_msg = None
    Twist_msg = Twist()
    Twist_msg.linear.x = 1.0
    new_turtle_response = None

    def __init__(self):
        # 3. NODE CONSTRUCTOR
        super().__init__('tutorial')
        self.counter = 0
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_sub_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.spawn_turtle = self.create_client(Spawn, "/spawn")

    # 4. NODE CALLBACKS
    def timer_callback(self):
        if self.new_turtle_response is None:
            self.spawn_req()
        time_now = self.get_clock().now().seconds_nanoseconds()
        self.counter += 1
        print(f'sec: {time_now[0]}, nanosec: {time_now[1]}', self.counter)
        print(f'({self.pose_msg.x}, {self.pose_msg.y})')
        self.alt_cmd_vel()
        if self.new_turtle_response.done():
            print(self.new_turtle_response.result().name)
        

    def pose_sub_callback(self, msg):
        self.pose_msg = msg

    # 5. HOW TO USE 
    def alt_cmd_vel(self):
        self.Twist_msg.linear.x *= -1
        self.cmd_pub.publish(self.Twist_msg)

    def spawn_req(self):
        request = Spawn.Request()
        request.name = ""
        request.x = 1.0
        request.y = 1.0
        self.new_turtle_response = self.spawn_turtle.call_async(request)

# MAIN BOILER PLATE
def main(args=None):
    rclpy.init(args=args)
    node = tutorial() # this changed
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()