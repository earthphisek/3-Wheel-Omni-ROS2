#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class RosToSerialNode(Node):

    def __init__(self):
        super().__init__('ros_to_serial_node')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.callback,
            10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200)  # Adjust serial port and baudrate
        self.get_logger().info('ROS to Serial Node initialized')
        
        
    def change_command_to_omega(self,x, y, z,):
        M_1 = y + 0.295 * z
        M_2 = 0.866 * x - 0.5 * y + 0.295 * z
        M_3 = -0.866 * x - 0.5 * y + 0.295 * z

        M_1 *= 16
        M_2 *= 16
        M_3 *= 16
        return M_1,M_2,M_3
        
    def change_to_count(self, omega1,omega2,omega3):
        delta_time = 10  # millisec
        count_per_second1 = 600 
        count_per_second2 = 1000  # from encoder
        sec_to_millisec = 1000  # sec_to_millisec
        big_round = 4.6  # cm
        small_round = 0.63  # cm
        pi = 3.1416
        
        count1 = (omega1 * delta_time * count_per_second1 * big_round) / (sec_to_millisec * small_round * 2 * pi)
        count2 = (omega2 * delta_time * count_per_second2 * big_round) / (sec_to_millisec * small_round * 2 * pi)
        count3 = (omega3 * delta_time * count_per_second2 * big_round) / (sec_to_millisec * small_round * 2 * pi)
        # log_msg = 'Received Twist data from ROS: count1 = %.2f, count2 = %.2f, count3 = %.2f' % (count1, count2, count3) 
        # self.get_logger().info(log_msg)
        return count1,count2,count3

    def callback(self, msg):
            linear_x = msg.linear.x
            linear_y = msg.linear.y
            angular_z = msg.angular.z
            M_1,M_2,M_3 = self.change_command_to_omega(linear_x, linear_y, angular_z)
            count1,count2,count3 = self.change_to_count(M_1,M_2,M_3)
            log_msg = 'Received Twist data from ROS: count1 = %.2f, count2 = %.2f, count3 = %.2f' % (count1, count2, count3) 

            # self.get_logger().info(log_msg)

            # Convert twist data to string format for serial transmission
            data_to_send = '{:.2f},{:.2f},{:.2f}\n'.format( count1,count2,count3)
            # data_to_send = '{},{},{}\n'.format( -10,-10,-10) # RIGHT
            # data_to_send = '{},{},{}\n'.format( 10,10,10) # LEFT
            
            try:
                self.serial_port.write(data_to_send.encode())  # Send data to microcontroller via serial
                # self.data = self.serial_port.readline()
                # self.get_logger().info(self.data) 
                
            except Exception as e:
                error_msg = 'Error sending data to microcontroller: %s' % str(e)
                self.get_logger().error(error_msg)

def main(args=None):
    rclpy.init(args=args)
    ros_to_serial_node = RosToSerialNode()
    rclpy.spin(ros_to_serial_node)
    ros_to_serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    time.sleep(0.01)
    main()


