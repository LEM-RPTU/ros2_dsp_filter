import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32


class SineWavePublisher(Node):

    def __init__(self):
        super().__init__('sine_wave_publisher')
        self.declare_parameter('frequency', 10.0)
        self.declare_parameter('amplitude', 10.0)
        self.declare_parameter('sample_time', 0.02)
        self.declare_parameter('topic_name', 'sine_wave')
        self.publisher = self.create_publisher(Float32, self.get_parameter('topic_name').get_parameter_value().string_value, 10)
        # 
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.amplitude = self.get_parameter('amplitude').get_parameter_value().double_value
        self.timer_period = self.get_parameter('sample_time').get_parameter_value().double_value  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.sample = 0

        if(self.frequency > 2/self.timer_period):
            self.get_logger().warn('Shannon-Nyquist theorem not satisfied. Consider increasing sample rate or decreasing frequency')

    def timer_callback(self):
        msg = Float32()
        msg.data = self.amplitude * np.sin(2 * np.pi * self.frequency * self.sample * self.timer_period)
        self.publisher.publish(msg)
        self.sample += 1


def main(args=None):
    rclpy.init(args=args)

    sine_wave_publisher = SineWavePublisher()

    rclpy.spin(sine_wave_publisher)

    sine_wave_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()