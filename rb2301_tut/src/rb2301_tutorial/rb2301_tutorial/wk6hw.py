# 1. IMPORT
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter as Param, ParameterType


# NODE CLASS
class parameter_services(Node):

    # 2. NODE PROPERTIES
    

    def __init__(self):
        # 3. NODE CONSTRUCTOR
        super().__init__('parameter_services')
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.set = self.create_client(SetParameters, "/turtlesim/set_parameters")
        self.get = self.create_client(GetParameters, "/turtlesim/get_parameters")
  

    # 4. NODE CALLBACKS
    def timer_callback(self):
        tpgr = GetParameters.Request()
        tpgr.names = ["background_b", "background_r"]
        self.gr = self.get.call_async(tpgr)
        self.gr.add_done_callback(self.grdone)
        

    # 5. HOW TO USE 
    def grdone(self,future):
        self.b = future.result().values[0].integer_value
        self.r = future.result().values[1].integer_value
        tpsr = SetParameters.Request()
        b = Param()
        b.name = 'background_b'
        b.value.type = ParameterType.PARAMETER_INTEGER
        b.value.integer_value = (self.b+50)%256
        r = Param()
        r.name = 'background_r'
        r.value.type = ParameterType.PARAMETER_INTEGER
        r.value.integer_value = (self.r - 20)%256
        tpsr.parameters = [b,r]
        self.sr = self.set.call_async(tpsr)
        print(f'Setting Red({r.value.integer_value:3d}) and Blue({b.value.integer_value:3d})')    


# MAIN BOILER PLATE
def main(args=None):
    rclpy.init(args=args)
    node = parameter_services() # this changed
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()