#!/usr/bin/env python3 

import rclpy
import numpy as np
import matplotlib.pyplot as plt
import rclpy.duration
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult, Parameter
import os
import yaml
import ros2topic.api
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from typing import Dict
from filter_signal.fourier_parameters import ParameterSet
# self-written stuff
import filter_signal.utilities as utilities

class FFTPlotter(Node):

    def __init__(self):
        super().__init__('fft_plotter')
        self.init_parameters()
        self.initialize_fft()
        self.message_type = ros2topic.api.get_msg_class(self, self.params.topic, blocking=True, include_hidden_topics=True)
        #QoS Settings should be compatible with whatever the publisher is doing, see:
        # https://docs.ros.org/en/iron/Concepts/Intermediate/About-Quality-of-Service-Settings.html#quality-of-service-settings
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        self.window_multipliers = [1.0]*self.params.number_of_samples
        self.fading_interval = 1
        self.jamm() 
        self.number_of_messages_received = 0
        self.got_first_message = True
        self.subscription = self.create_subscription(self.message_type, self.params.topic, self.subscription_callback, qos_profile=self.qos_profile)

    def load_filter_config(self) -> Dict:
        path = os.path.join(self.params.config_path, self.params.config_file_name + '.yaml')
        try:
            with open(path, "r") as file:
                self.get_logger().info('Successfully loaded file: {}'.format(path))
                return yaml.safe_load(file) 
        except FileNotFoundError:
            self.get_logger().error('Could not load file: {}. Check path validity.'.format(path))
            return None

    def initialize_fft(self):
        filter_config = self.load_filter_config()
        self.key_signal_buffer_pairs = list()
        if filter_config is None:
            return None
        else:
            for key, value in utilities.iterate_leafs(filter_config, parent_key=''):
                if type(value) == bool:
                    if value == True:
                        self.key_signal_buffer_pairs.append([key, [0.0] * self.params.number_of_samples])
                    else:
                        continue
                else:
                    continue
            self.get_logger().info("Will perform Fourier Transforms of signals at keys:")
            for key_and_buffer in self.key_signal_buffer_pairs:
                self.get_logger().info(f"{key_and_buffer[0]}")
            return None


    def init_parameters(self):
        self.declare_parameter('topic', 'sample_signal')
        self.declare_parameter('number_of_samples', 1000)
        self.declare_parameter('config_path', os.path.expanduser("~"))
        self.declare_parameter('config_file_name', 'float32')
        # Read values of parameters
        self.params = ParameterSet(
            topic = self.get_parameter('topic').get_parameter_value().string_value,
            number_of_samples = self.get_parameter('number_of_samples').get_parameter_value().integer_value,
            config_path = self.get_parameter('config_path').get_parameter_value().string_value,
            config_file_name = self.get_parameter('config_file_name').get_parameter_value().string_value
        )
        for parameter in self.params.__dict__.keys():
            self.get_logger().info(f"Parameter {parameter} is: {self.params.__dict__[parameter]}")
        self.add_on_set_parameters_callback(self.parametersCallback)

    def parametersCallback(self, parameters: list[Parameter]):
        result = SetParametersResult()
        result.successful = True
        for param in parameters:
            if param.name == 'topic':
                result.successful = False
                result.reason = f"Parameter {param.name} cannot be changed at runtime."
                self.get_logger().warn(result.reason)
            elif param.name == 'number_of_samples':
                result.successful = False
                result.reason = f"Parameter {param.name} cannot be changed at runtime."
                self.get_logger().warn(result.reason)
            elif param.name == 'config_path':
                result.successful = False
                result.reason = f"Parameter {param.name} cannot be changed at runtime."
                self.get_logger().warn(result.reason)
            elif param.name == 'config_file_name':
                result.successful = False
                result.reason = f"Parameter {param.name} cannot be changed at runtime."
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

    def subscription_callback(self, msg):
        if(self.got_first_message):
            self.time_at_first_message = self.get_clock().now()
            self.got_first_message = False
        for pair in self.key_signal_buffer_pairs:
            current_key = pair[0]
            value = utilities.get_nested_Attr(msg, current_key)
            if type(value) == float:    
                pair[1][self.number_of_messages_received] = value * self.window_multipliers[self.number_of_messages_received]
            else:
                self.get_logger().error(f"Value of key: {current_key} is not a float.")
        self.number_of_messages_received += 1
        if self.number_of_messages_received >= self.params.number_of_samples:
            self.time_difference = self.get_clock().now() - self.time_at_first_message
            self.time = float(self.time_difference.nanoseconds / 1e9)  # Convert nanoseconds to milliseconds 
            self.get_logger().info(f'Duration: {self.time} s')
            self.sample_frequency = self.params.number_of_samples/self.time
            self.get_logger().info(f'Sample Frequency: {self.sample_frequency:.6f} Hz')
            self.nyquist_frequency = self.sample_frequency/2
            for key_signal_buffer_pair in self.key_signal_buffer_pairs:
                self.perform_fft(key_signal_buffer_pair[0], key_signal_buffer_pair[1])
            plt.show()
        return

    def perform_fft(self, key, signal_buffer):
        signal = np.array(signal_buffer)
        N = len(signal)  # Number of samples
        T = 1/self.sample_frequency  # Sample interval

        fft_result = np.fft.fft(signal)
        frequencies = np.fft.fftfreq(N, T)  # Frequency in Hz
        # Normalize the FFT amplitude and scale
        fft_amplitude = 2.0 / N * np.abs(fft_result[:N // 2])

        # Only plot the positive half of the frequencies
        plt.figure()
        plt.plot(frequencies[:N // 2], fft_amplitude)
        plt.title(f'{self.get_namespace()}/{self.params.topic}:{key}')
        plt.xlabel('Frequency/Hz')
        plt.ylabel('Amplitude')
        plt.grid()
        
        

        


def main(args=None):
    rclpy.init(args=args)
    fastFourier_plotter = FFTPlotter()
    rclpy.spin(fastFourier_plotter)
    fastFourier_plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
