#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


class IMUPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')

        self.declare_parameter('subscription_topic',    "sample_wave")
        self.declare_parameter('publisher_topic',       "imu_sample")
        self.declare_parameter('message_component',     "linear_acceleration_z")

        self.component = self.get_parameter('message_component').get_parameter_value().string_value   

        self.subscription = self.create_subscription(Float32, self.get_parameter('subscription_topic').get_parameter_value().string_value, self.subscription_callback, 10)
        self.publisher = self.create_publisher(Imu, self.get_parameter('publisher_topic').get_parameter_value().string_value, 10)

    def subscription_callback(self, msg):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_frame'
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0
        
        if self.component == "orientation_x": 
            imu_msg.orientation.x = msg.data 
        elif self.component == "orientation_y":
            imu_msg.orientation.y = msg.data 
        elif self.component == "orientation_z": 
            imu_msg.orientation.z = msg.data 
        elif self.component == "orientation_w":
            imu_msg.orientation.w = msg.data
        elif self.component == "angular_velocity_x": 
            imu_msg.angular_velocity.x = msg.data 
        elif self.component == "angular_velocity_y": 
            imu_msg.angular_velocity.y = msg.data 
        elif self.component == "angular_velocity_z": 
            imu_msg.angular_velocity.z = msg.data 
        elif self.component == "linear_acceleration_x": 
            imu_msg.linear_acceleration.x = msg.data 
        elif self.component == "linear_acceleration_y": 
            imu_msg.linear_acceleration.y = msg.data 
        elif self.component == "linear_acceleration_z": 
            imu_msg.linear_acceleration.z = msg.data 
        else: self.get_logger().warn(f"Unknown message component: {self.component}")

        self.publisher.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)

    imu_publisher = IMUPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()