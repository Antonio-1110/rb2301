import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class sim(Node):
    odom_msg = None
    def __init__(self):
        super().__init__('sim')
        self.timer = self.create_timer(0.2,self.t_callback)
        self.sub = self.create_subscription(Odometry, '/odom', self.sub_callback,10)
        

    def t_callback(self):
        if self.odom_msg:
            print(f"x = {self.odom_msg.pose.pose.position.y}, y = {self.odom_msg.pose.pose.position.y}")
    
    def sub_callback(self, msg):
        self.odom_msg = msg

def main(args=None):
    rclpy.init(args=args)
    node = sim() # this changed
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()