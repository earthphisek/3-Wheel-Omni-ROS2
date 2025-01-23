#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Duration

from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist, Pose, PoseStamped, Quaternion ,TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import numpy as np
import time
import math 

def quaternion_from_euler(ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q

class MinimalSubscriber(Node):

    

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/int32_encoder_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.update_rate = 10  # Hz
        self.update_period = 1.0 / self.update_rate
        self.timer = self.create_timer(self.update_period, self.publish_odometry)
        self.current_time = self.get_clock().now()
        self.last_time = self.get_clock().now()


        self.Px_0 = 0.0
        self.Py_0 = 0.0
        self.theta_0 = 0.0
        self.Px = 0.0
        self.Py = 0.0
        self.theta_delta = 0.0
        self.deltaT = 0.01
        # Initialize odometry values
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.Vx = 0.0
        self.Vy = 0.0
        self.w = 0.0

        self.omega_1 = 0.0
        self.omega_2 = 0.0
        self.omega_3 = 0.0
        self.wheel_radius = 0.075
        self.pulse_per_radius = 0.000196 
        self.robot_length = 0.295
        
        self.t = time.time()
        


    def listener_callback(self, msg):

        # Your code goes here
        # self.dt = (self.current_time - self.last_time)
        # self.get_logger().info('I heard: "%s "' %(self.dt))
        # self.encoder_1 = msg.data[0]
        # self.encoder_2 = msg.data[1]
        # self.encoder_3 = msg.data[2]
        # self.phi_1 = self.pulse_per_radius*self.encoder_1
        # self.phi_2 = self.pulse_per_radius*self.encoder_2
        # self.phi_3 = self.pulse_per_radius*self.encoder_3
        # self.X = self.wheel_radius*(2/3*self.phi_1 - 1/3*self.phi_2 -1/3*self.phi_3)
        # self.Y = self.wheel_radius*((1/math.sqrt(3))*self.phi_2) - (1/math.sqrt(3))*self.phi_3
        # self.th = (self.wheel_radius/3*self.robot_length)*(self.phi_1+self.phi_2+self.phi_3)
        
        # print(time.time() - self.t)
        self.t = time.time()
        self.omega_1 = msg.data[0] * 0.0484 * (47/60) / 0.6
        self.omega_2 = msg.data[1] * 0.0484 * (47/60)
        self.omega_3 = msg.data[2] * 0.0484 * (47/60)
        
        print(str(self.omega_1) + "\t" + str(self.omega_2) + "\t" + str(self.omega_3))
        self.Vx = -(-0.5774*self.omega_2 + 0.5774*self.omega_3)
        self.Vy = -(0.667*self.omega_1 - 0.3333*self.omega_2 - 0.3333*self.omega_3)
        self.w = 1.1299*self.omega_1 + 1.1299*self.omega_2 + 1.1299*self.omega_3
        
        self.Px =  self.Vx*math.cos(self.theta)*self.deltaT +  self.Vy*math.cos(math.pi/2 + self.theta)*self.deltaT
        self.Py =  self.Vx*math.sin(self.theta)*self.deltaT +  self.Vy*math.sin(math.pi/2 + self.theta)*self.deltaT
        self.theta_delta =  self.w*self.deltaT
        
        # print(str(self.Px) + "\t" + str(self.Py) + "\t" + str(self.theta_delta))
        # self.Px = (Vx*math.cos(self.theta) + Vy*math.cos(math.pi/2 + self.theta))*self.deltaT
        # self.Py = (Vx*math.sin(self.theta) + Vy*math.sin(math.pi/2 + self.theta))*self.deltaT
        # self.theta_delta = w*self.deltaT

        # self.get_logger().info('I heard: "%s %s %s "' %(self.Px,self.Py,self.theta_delta))

        # self.get_logger().info('I heard: "%s %s %s "' %(Vx,Vy,w))
        # self.last_time = self.current_time
    



    def publish_odometry(self):
        self.x += self.Px
        self.y += self.Py
        self.theta += self.theta_delta


        # self.x += math.cos(self.theta)*self.X - math.sin(self.theta)*self.Y
        # self.y += math.sin(self.theta)*self.X - math.cos(self.theta)*self.Y
        # self.theta += self.th

        # Publish the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        # Set the pose information
        # pose = Pose()
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        # pose.orientation = Quaternion(x=0.0, y=0.0, z=math.sin(self.theta / 2), w=math.cos(self.theta / 2))
        q = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # print(str(self.omega_1) + "\t" + str(self.omega_2) + "\t" + str(self.omega_3))

        # Set the twist information
        odom_msg.twist.twist.linear.x = self.Vx * 0.833/10.0
        odom_msg.twist.twist.linear.y = self.Vy * 0.833/10.0
        odom_msg.twist.twist.angular.z = self.w * 0.833/10.0

        self.odom_pub.publish(odom_msg)
        
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        # q = quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()