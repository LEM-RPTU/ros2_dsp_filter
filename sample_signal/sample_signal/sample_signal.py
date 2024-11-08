from rclpy.node import Node
from std_msgs.msg import Float32

import numpy as np
import rclpy


class SampleWavePublisher(Node):

    def __init__(self):
        super().__init__('sample_wave_publisher')
        
        self.declare_parameter('sine_wave_components',          [2.0, 40.0, 1.57])
        self.declare_parameter('publisher_frequency',            50.0)
        self.declare_parameter('base_signal_initial_value',      0.0)
        self.declare_parameter('base_signal_step',               0.1)

        self.timer_period = 1/self.get_parameter('publisher_frequency').get_parameter_value().double_value
        self.sine_wave_components = self.get_parameter('sine_wave_components').get_parameter_value().double_array_value
        self.base_signal_value = self.get_parameter('base_signal_initial_value').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(Float32, 'sample_wave', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.time_step = 0.0

    def timer_callback(self): 
        msg = Float32() 
        msg.data = 0.0

        for i in range(0, len(self.sine_wave_components), 3): 
            sine_frequency = self.sine_wave_components[i] 
            sine_amplitude = self.sine_wave_components[i + 1] 
            phase_shift =    self.sine_wave_components[i + 2] 
            msg.data += sine_amplitude * np.cos(2 * np.pi * sine_frequency * self.time_step * self.timer_period + phase_shift) + self.base_signal_value
            
        self.publisher_.publish(msg) 
        self.time_step += 1.0
        self.base_signal_value += self.get_parameter('base_signal_step').get_parameter_value().double_value


def main(args=None):
    rclpy.init(args=args)
    sample_wave_publisher = SampleWavePublisher()
    rclpy.spin(sample_wave_publisher)
    sample_wave_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()