#!/usr/bin/python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from std_msgs.msg import Int32MultiArray
import yaml
import math
class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')
        self.command_publisher = self.create_publisher(Float64MultiArray,'/velocity_controllers/commands',10)
        self.encoder_subscription = self.create_subscription(Int32MultiArray,'/int32_encoder_data',self.encoder_callback,10)
        # self.period = 0.1
        # self.timer = self.create_timer(self.period,self.timer_callback)
        self.counter = 0
        self.wheel_radius = 0.075
        # self.get_logger().info(f'WR:{self.encoder_subscription}')
    # def timer_callback(self):
    #     hold_time = 0.5
    #     if self.counter < hold_time:
    #         self.counter = self.counter + self.period
    #         if self.counter >=hold_time:
    #             cmd = Float64MultiArray()
    #             cmd.data = [0.0,0.0]
    #             self.command_publisher.publish(cmd)
    def encoder_callback(self,msg:Float32MultiArray):
        cmd = Float32MultiArray()
        # self.get_logger().info(cmd)
        # cmd.data = self.compute(msg.data[0],msg.data[1],msg.data[2])
        self.counter = 0
        self.command_publisher.publish(cmd)
    def compute(self,vx,vy,w):

        # A_wheel_velocity = ((w*self.wheel_radius) - vy)
        # B_wheel_velocity = (w*self.wheel_radius + (math.sqrt(3)*vx/2) + (vy/2))
        # C_wheel_velocity = (w*self.wheel_radius - (math.sqrt(3)*vx/2) + (vy/2))

        # left_wheel_velocity = v/self.wheel_radius-w*self.wheel_separation/(2*self.wheel_radius)
        # right_wheel_velocity = v/self.wheel_radius+w*self.wheel_separation/(2*self.wheel_radius)        
        # return [A_wheel_velocity,B_wheel_velocity,C_wheel_velocity]
        return 

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
