#!/usr/bin/env python3 

import rclpy
import numpy as np
import matplotlib.pyplot as plt
import rclpy.duration
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult, Parameter

from fft.parameters import ParameterSet

class FFTPlotter(Node):

    def __init__(self):
        super().__init__('fft_plotter')
        self.init_parameters()
        # Create a subscription to the topic
        self.subscription = self.create_subscription(Float32, self.params.topic, self.listener_callback, 10)
        self.window_multipliers = [1.0]*self.params.number_of_samples
        self.fading_interval = 1
        self.jamm() 
        self.signal_buffer = []
        self.counter = 0
        self.subscription
        self.got_first_message = True


    def init_parameters(self):
        self.declare_parameter('topic', 'sample_wave')
        self.declare_parameter('number_of_samples', 1000)
        # Read values of parameters
        self.params = ParameterSet(
            topic = self.get_parameter('topic').get_parameter_value().string_value,
            number_of_samples = self.get_parameter('number_of_samples').get_parameter_value().integer_value
        )
        
        self.add_on_set_parameters_callback(self.parametersCallback)

    def parametersCallback(self, parameters: list[Parameter]):
        result = SetParametersResult()
        result.successful = True
        for param in parameters:
            if param.name == 'topic':
                result.successful = False
                result.reason = f"Parameter {param.name} cannot be changed runtime."
                self.get_logger().warn(result.reason)
            elif param.name == 'number_of_samples':
                result.successful = False
                result.reason = f"Parameter {param.name} cannot be changed runtime."
                self.get_logger().warn(result.reason)
            else:
                result.successful = False
                result.reason = f"Could not find {param.name} to be changed or parameter is not implemented for run-time change."
                self.get_logger().warn(result.reason)             
        return result

    def jamm(self):
        fade_in = np.linspace(0, 1, self.fading_interval)
        fade_out = np.linspace(1, 0, self.fading_interval)
        self.window_multipliers[:self.fading_interval] = fade_in
        self.window_multipliers[-self.fading_interval:] = fade_out

    def listener_callback(self, msg):
        if(self.got_first_message):
            self.start_time = self.get_clock().now()
            self.got_first_message = False
        self.signal_buffer.append(msg.data * self.window_multipliers[self.counter])
        self.counter = self.counter + 1
        if len(self.signal_buffer) >= self.params.number_of_samples:
            self.time_difference = self.get_clock().now() - self.start_time
            self.time = float(self.time_difference.nanoseconds / 1e9)  # Convert nanoseconds to milliseconds 
            self.get_logger().info(f'Duration: {self.time} s')
            self.sample_frequency = self.params.number_of_samples/self.time
            self.get_logger().info(f'Sample Frequency: {self.sample_frequency:.6f} Hz')
            self.nyquist_frequency = self.sample_frequency/2
            self.perform_fft()
            self.signal_buffer.clear()
    def perform_fft(self):
        signal = np.array(self.signal_buffer)
        N = len(signal)  # Number of samples
        T = 1/self.sample_frequency  # Sample interval

        fft_result = np.fft.fft(signal)
        frequencies = np.fft.fftfreq(N, T)  # Frequency in Hz
        self._logger
        # Normalize the FFT amplitude and scale
        fft_amplitude = 2.0 / N * np.abs(fft_result[:N // 2])

        # Only plot the positive half of the frequencies
        plt.plot(frequencies[:N // 2], fft_amplitude)
        plt.title(f'{self.params.topic}')
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('Amplitude')
        plt.grid()
        plt.show()

        self.get_logger().info(f'FFT computed for signal with {N} samples')

        


def main(args=None):
    rclpy.init(args=args)
    fastFourier_plotter = FFTPlotter()
    rclpy.spin(fastFourier_plotter)
    fastFourier_plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
