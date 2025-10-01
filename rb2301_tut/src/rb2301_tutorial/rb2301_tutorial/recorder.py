import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class Recorder(Node):

    def __init__(self):
        super().__init__('recorder')
        self.f = open('recorder.txt','w')
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_sub_callback, 10)
        

    def timer_callback(self):
        pass

    def pose_sub_callback(self,msg):
        self.f.write(f'{msg.x}\t{msg.y}\t{msg.theta}\n')







def main(args=None):
    rclpy.init(args=args)
    node = Recorder() # this changed
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()