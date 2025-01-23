import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time
import tuya_fngerbot
from playsound import playsound
import sys



MIN_DISTANCE_TO_MOVE = 1.2  # Minimum distance to move the robot (in meters)i
ROTATION_DURATION = 7.0  # Rotation duration in seconds
FORWARD_DURATION = 12.0
FORWARD_DURATION_OUT = 12.0
FORWARD_SPEED = 0.15
ROTATION_SPEED = -0.30  # Angular speed for rotationฟ
DELAY_TUYA_DURATION = 1.0



class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd = Twist()
        self.rotation_start_time = None
        self.forward_start_time = None
        self.delay_tuya_time = None
        self.moving_forward = False
        self.delay_tuya = False
        self.moving_Rotate = False
        self.moving_Out = False
        self.check_gate = True
        self.check_gate_out = False
        tuya_fngerbot.select_fingerbot("Zigbee_2")
        playsound("/home/earththesis/ros2_directory/3wheel_ws/draft.mp3")
        

    def scan_callback(self, msg):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # Calculate the index corresponding to 0 degrees
        zero_index = int(-angle_min / angle_increment)

        # Get the distance at 0 degrees
        distance_at_zero = msg.ranges[zero_index]
        # self.get_logger().info(f'{distance_at_zero}')
        
        if self.check_gate:
            # self.get_logger().info(f'{distance_at_zero}')
            if distance_at_zero > MIN_DISTANCE_TO_MOVE:
                self.moving_forward = True
                self.check_gate = False
                # self.get_logger().info(f'{self.moving_forward}')
            
        # Check if the distance is valid and more than MIN_DISTANCE_TO_MOVE
        if self.moving_forward:
            if self.forward_start_time is None:
                self.forward_start_time = time.time()
                self.get_logger().info(f'get in')
                tuya_fngerbot.select_fingerbot("BLE_5")
                playsound("/home/earththesis/ros2_directory/3wheel_ws/draft1.mp3")

            elif time.time() - self.forward_start_time < FORWARD_DURATION:
                # self.get_logger().info(f'Forward for {FORWARD_DURATION} seconds...')

                # Set the linear velocity to move the robot forward
                self.cmd.linear.x = FORWARD_SPEED  # Adjust the speed as needed
            else:
                self.get_logger().info('Forward completed. Stopping rotation.')

                # Stop the robot by setting linear velocity to 0
                self.cmd.linear.x = 0.0
                self.moving_forward = False
                
                tuya_fngerbot.select_fingerbot("BLE_2")
                playsound("/home/earththesis/ros2_directory/3wheel_ws/draft2.mp3")
                self.delay_tuya = True
        
        if self.delay_tuya:
            if self.delay_tuya_time is None:
                self.delay_tuya_time = time.time()
            elif time.time() - self.delay_tuya_time < DELAY_TUYA_DURATION:
                pass
            else:
                tuya_fngerbot.select_fingerbot("BLE_4")
                self.delay_tuya = False
                self.moving_Rotate = True
                
            
                
                
                
        if self.moving_Rotate:
                if self.rotation_start_time is None:
                    self.rotation_start_time = time.time()
                elif time.time() - self.rotation_start_time < ROTATION_DURATION:
                    self.cmd.angular.z = ROTATION_SPEED  # Rotate in place
                else:
                    self.cmd.angular.z = 0.0
                    self.moving_Rotate = False
                    playsound("/home/earththesis/ros2_directory/3wheel_ws/draft3.mp3")
                    self.check_gate_out = True
                    self.rotation_start_time = None
                    self.forward_start_time = None
                    
        
                       
        if self.check_gate_out:
            if distance_at_zero > 3.0:
                self.forward_start_time = None
                self.moving_Out = True
                self.check_gate_out = False
                playsound("/home/earththesis/ros2_directory/3wheel_ws/draft4.mp3")
                
                    
        if self.moving_Out:
            if self.forward_start_time is None:
                self.forward_start_time = time.time()
                self.get_logger().info(f'get in')
                # tuya_fngerbot.select_fingerbot("BLE_5")
               

            elif time.time() - self.forward_start_time < FORWARD_DURATION_OUT:
                self.cmd.linear.x = 0.15  # Adjust the speed as needed
            else:
                self.get_logger().info('Forward completed. Stopping rotation.')

                self.cmd.linear.x = 0.0
                self.cmd.linear.x = 0.0
                self.moving_Out = False
                self.forward_start_time = None
                tuya_fngerbot.select_fingerbot("BLE_3")
                playsound("/home/earththesis/ros2_directory/3wheel_ws/draft5.mp3")
                # sys.exit("Complete")
                
        #  if self.moving_forward:
        #     if distance_at_zero > MIN_DISTANCE_TO_MOVE:
        #         self.get_logger().info(f'Distance at 0 degrees: {distance_at_zero} meters')
        #         self.get_logger().info('Distance at 0 degrees is more than 0.3 meters. Moving robot forward.')

        #         # Set the linear velocity to move the robot forward
        #         self.cmd.linear.x = 0.12  # Adjust the speed as needed
        #     else:
        #         self.get_logger().info(f'Distance at 0 degrees: {distance_at_zero} meters')
        #         self.get_logger().info('Distance at 0 degrees is less than or equal to 0.3 meters. Stopping robot.')

        #         # Stop the robot by setting linear velocity to 0
        #         self.cmd.linear.x = 0.0
        #         self.moving_forward = False
            
             # Rotate the robot if it was previously moving forward
                    
            
            
        
        # self.cmd.linear.x = 0.0
        # self.cmd.angular.z = 0.0
        # Publish the velocity command
        self.publisher.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
