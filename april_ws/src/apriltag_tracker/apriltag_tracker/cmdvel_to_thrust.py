import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class CmdVelToThrust(Node):
    def __init__(self):
        super().__init__('cmdvel_to_thrust')

        self.port_pub = self.create_publisher(
            Float64,
            '/model/blueboat/joint/motor_port_joint/cmd_thrust',
            10
        )
        self.stbd_pub = self.create_publisher(
            Float64,
            '/model/blueboat/joint/motor_stbd_joint/cmd_thrust',
            10
        )

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

    def cmd_vel_cb(self, msg):
        linear  = msg.linear.x   # surge
        angular = msg.angular.z  # yaw

        
        port_thrust = linear - angular
        stbd_thrust = linear + angular
        
        if linear < 0:
            angular = -angular


        
        #scale = 3.5
        
        scale_linear  = 3.5
        scale_angular = 3.5
        port_thrust = (linear * scale_linear) - (angular * scale_angular)
        stbd_thrust = (linear * scale_linear) + (angular * scale_angular)

        port_msg = Float64()
        stbd_msg = Float64()
        port_msg.data = port_thrust
        stbd_msg.data = stbd_thrust

       
        
        # port_msg = Float64()
        # stbd_msg = Float64()
        # port_msg.data = port_thrust * scale
        # stbd_msg.data = stbd_thrust * scale

        self.port_pub.publish(port_msg)
        self.stbd_pub.publish(stbd_msg)

def main():
    rclpy.init()
    node = CmdVelToThrust()
    rclpy.spin(node)

if __name__ == '__main__':
    main()