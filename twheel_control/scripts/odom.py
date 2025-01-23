import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist, Pose, PoseStamped, Quaternion,TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import tf_transformations 
import math 
class OdomPublisher(Node):

    def __init__(self):
        super().__init__('odom_publisher')
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
        self.trans_x = 0.0
        self.trans_y = 0.0
        self.rot_z = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_th = 0.0
        self.pose_quat = tf_transformations.quaternion_from_euler(0, 0, self.pose_th)
        self.current_time = self.get_clock().now()
        self.previous_time = self.get_clock().now()
        self.distance_robot = 0.295
        self.radius_wheel = 0.075

    def normalize_theta(self, theta):
        return math.atan2(math.sin(theta), math.cos(theta))


    def listener_callback(self, msg):
        self.current_time = self.get_clock().now()
        self.u_1 = msg.data[0]* 0.0484 * (47/60) / 0.6
        self.u_2 = msg.data[1]* 0.0484 * (47/60)
        self.u_3 = msg.data[2]* 0.0484 * (47/60)

        self.rot_z = (self.u_2-self.distance_robot*self.u_1)/self.radius_wheel
        self.trans_x = -(self.u_2 + (1.732*self.u_3) + (2*self.distance_robot*self.u_1)) / 2*self.radius_wheel
        self.trans_y = -((1.732*1.732*(self.u_2)) - (3*self.u_3) + (2*1.732*self.distance_robot*self.u_1)) / (6*self.radius_wheel)
        
        dt = (self.current_time - self.previous_time).to_msg()
        # self.get_logger().info('I heard: "%s  "' %(dt.nanosec/math.pow(10,9)))
        self.previous_time = self.current_time
        self.pose_x += ((math.cos(self.pose_th) * self.trans_x - math.sin(self.pose_th) * self.trans_y) * (dt.nanosec/math.pow(10,9)))
        self.pose_y += ((math.sin(self.pose_th) * self.trans_x + math.cos(self.pose_th) * self.trans_y) * (dt.nanosec/math.pow(10,9)))
        angular = self.rot_z * dt.sec/math.pow(10,9)
        self.pose_th = self.normalize_theta(self.pose_th + angular)

        self.pose_quat = tf_transformations.quaternion_from_euler(0, 0, self.pose_th)

        # self.get_logger().info('I heard: "%s %s %s "' %(self.pose_x,self.pose_y,self.pose_th))
        

    def publish_odometry(self):
       

        # self.x += self.Px
        # self.y += self.Py
        # self.theta += self.th

        # Publish the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = self.pose_x/100
        odom_msg.pose.pose.position.y = self.pose_y/100
        odom_msg.pose.pose.orientation.x = self.pose_quat[0]
        odom_msg.pose.pose.orientation.y = self.pose_quat[1]
        odom_msg.pose.pose.orientation.z = self.pose_quat[2]
        odom_msg.pose.pose.orientation.w = self.pose_quat[3]
        odom_msg.twist.twist.linear.x = self.trans_x
        odom_msg.twist.twist.linear.y = self.trans_y
        odom_msg.twist.twist.angular.z = self.rot_z

        self.odom_pub.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.pose_x/100
        t.transform.translation.y = self.pose_y/100
        t.transform.translation.z = 0.0
        t.transform.rotation.x = self.pose_quat[0]
        t.transform.rotation.y = self.pose_quat[1]
        t.transform.rotation.z = self.pose_quat[2]
        t.transform.rotation.w = self.pose_quat[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

        




def main(args=None):
    rclpy.init(args=args)

    odom_publisher = OdomPublisher()

    rclpy.spin(odom_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()