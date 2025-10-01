# 1. IMPORT
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

# NODE CLASS
class wk6node(Node):

    # 2. NODE PROPERTIES
    

    def __init__(self):
        # 3. NODE CONSTRUCTOR
        super().__init__('parameters')
        self.declare_parameter('values.bool_v', False)
        self.declare_parameter('values.int_v', 0)
        self.declare_parameter('values.dbl_v', 0.0)
        self.declare_parameter('values.str_v', '')
        self.declare_parameter('bool_arr_v', [False])
        self.declare_parameter('int_arr_v', [0])
        self.declare_parameter('dbl_arr_v', [0.0])
        self.declare_parameter('str_arr_v', [''])
        # self.declare_parameters(
        #     '',
        #     [
        #         ('values.bool_v', False),
        #         ('values.int_v', 0),
        #         ('values.dbl_v', 0.0),
        #         ('values.str_v', ''),
        #         ('bool_arr_v', [False]),
        #         ('int_arr_v', [0]),
        #         ('dbl_arr_v', [0.0]),
        #         ('str_arr_v', [''])

        #     ]
        # )
        self.add_on_set_parameters_callback(self.set_parameters_callbacks)
        self.timer = self.create_timer(1, self.timer_callback)

    # 4. NODE CALLBACKS
    def set_parameters_callbacks(self, parameter_list):
        for p in parameter_list:
            print(f'Setting {p.name}: {p.value}')
        return SetParametersResult(successful=True)
    def timer_callback(self):
        print(f'values.str_v : {self.get_parameter('values.str_v').value}')
        self.get_logger().info(f'{self.get_parameter('values.str_v').value}')
   

    # 5. HOW TO USE 
    

# MAIN BOILER PLATE
def main(args=None):
    rclpy.init(args=args)
    node = wk6node() # this changed
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()