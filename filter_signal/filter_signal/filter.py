#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult, Parameter
from filter_signal.filter_parameters import ParameterSet
from scipy import signal
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import ros2topic.api
import yaml
from typing import Dict
# self-written stuff
import filter_signal.filters as filters
import filter_signal.utilities as utilities

class SignalFilter(Node):

    def __init__(self):
        super().__init__('filter')
        self.init_parameters()
        self.message_type = ros2topic.api.get_msg_class(self, self.params.signal_topic, blocking=True, include_hidden_topics=True)
        #QoS Settings should be compatible with whatever the publisher is doing, see:
        # https://docs.ros.org/en/iron/Concepts/Intermediate/About-Quality-of-Service-Settings.html#quality-of-service-settings
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        self.number_of_messages_received = 0
        self.sample_time_subscription = self.create_subscription(self.message_type, self.params.signal_topic, self.sample_time_subscription_callback_, qos_profile=self.qos_profile)

    def sample_time_subscription_callback_(self, msg):
        if(self.number_of_messages_received == 0):
            self.get_logger().info(f"Started sample-time subscription.")
            self.time_first_message_received = self.get_clock().now()
            self.number_of_messages_received += 1
            return
        elif(self.number_of_messages_received == self.params.number_of_messages_to_average_sample_time_over):
            # calulate sample time & then destroy the subscription
            self.sample_time = (self.get_clock().now() - self.time_first_message_received).nanoseconds / (1e9*self.number_of_messages_received)
            self.get_logger().info(f"Destroying sample-time subscription.")
            self.destroy_subscription(self.sample_time_subscription)
            self.sample_frequency = (1.0/self.sample_time)
            self.get_logger().info(f"Sample time is: {self.sample_time:.4f} s.")
            self.get_logger().info(f"Sample frequency is: {self.sample_frequency:.4f} Hz.")
            # Initialize filters
            self.initialize_filters()
            # start subscribing signal and publishing filtered signal
            self.get_logger().debug(f"Starting publisher of filtered message.")
            self.publisher = self.create_publisher(self.message_type, self.params.signal_topic + "/filtered", qos_profile=self.qos_profile)
            self.get_logger().debug(f"Starting subscription for filtering.")
            self.subscription = self.create_subscription(self.message_type, self.params.signal_topic, self.subscription_callback, qos_profile=self.qos_profile)
            return
        else:
            self.number_of_messages_received += 1
            return


    def subscription_callback(self, msg):
        #pair = list([key, b, a ,filter, value])
        for pair in self.filter_key_pairs:
            current_key = pair[0]
            value = utilities.get_nested_Attr(msg, current_key)
            if type(value) == float:    
                filtered_value, pair[3] = signal.lfilter(pair[1], pair[2], [value], zi=pair[3])
                utilities.set_nested_Attr(msg, current_key, filtered_value[0])
            else:
                self.get_logger().error(f"Value of key: {current_key} is not a float.")
        self.publisher.publish(msg)

    def load_filter_config(self) -> Dict:
        path = os.path.join(self.params.config_path, self.params.config_file_name + '.yaml')
        try:
            with open(path, "r") as file:
                self.get_logger().error('Successfully loaded file: {}'.format(path))
                return yaml.safe_load(file) 
        except FileNotFoundError:
            self.get_logger().error('Could not load file: {}. Check path validity.'.format(path))
            return None
        
    def initialize_filters(self):
        filter_config = self.load_filter_config()
        self.filter_key_pairs = list()
        if filter_config is None:
            return None
        else:
            self.get_logger().info("Loaded parameter file: ")
            for key, value in utilities.iterate_leafs(filter_config, parent_key=''):
                if type(value) == list:
                    if value[0] == 'lowpass':
                        [b, a, filter] = filters.lowpass(frequency=value[1], order=value[2], sample_frequency=self.sample_frequency)
                        self.filter_key_pairs.append([key, b, a, filter, value])
                    elif value[0] == 'highpass':
                        [b, a, filter] = filters.highpass(frequency=value[1], order=value[2], sample_frequency=self.sample_frequency)
                        self.filter_key_pairs.append([key, b, a, filter, value])
                    elif value[0] == 'bandpass':
                        [b, a, filter] = filters.bandpass(frequency=value[1], order=value[2], sample_frequency=self.sample_frequency)
                        self.filter_key_pairs.append([key, b, a, filter, value])
                    elif value[0] == 'bandstop':
                        [b, a, filter] = filters.bandstop(frequency=value[1], order=value[2], sample_frequency=self.sample_frequency)
                        self.filter_key_pairs.append([key, b, a ,filter, value])
                    else:
                        continue
                else:
                    continue
            self.get_logger().info("Filters initialized with:")
            for key, b, a, filter, value in self.filter_key_pairs:
                self.get_logger().info(f"{key} with filter parameters: {value}.")
            return None
                
    def init_parameters(self):
        self.declare_parameter('signal_topic', 'imu')
        self.declare_parameter('config_path', os.path.expanduser("~"))
        self.declare_parameter('config_file_name', 'imu')
        self.declare_parameter('number_of_messages_to_average_sample_time_over', 1000)
        # Read values of parameters
        self.params = ParameterSet(
            signal_topic = self.get_parameter('signal_topic').get_parameter_value().string_value,
            config_path = self.get_parameter('config_path').get_parameter_value().string_value,
            config_file_name = self.get_parameter('config_file_name').get_parameter_value().string_value,
            number_of_messages_to_average_sample_time_over = self.get_parameter('number_of_messages_to_average_sample_time_over').get_parameter_value().integer_value
        )
        for parameter in self.params.__dict__.keys():
            self.get_logger().info(f"Parameter {parameter} is: {self.params.__dict__[parameter]}")

        self.add_on_set_parameters_callback(self.parametersCallback)

    def parametersCallback(self, parameters: list[Parameter]):
        result = SetParametersResult()
        result.successful = True
        for param in parameters:
            if param.name == 'signal_topic':
                result.successful = False
                result.reason = f"Parameter {param.name} cannot be changed runtime."
                self.get_logger().warn(result.reason)
            elif param.name == 'config_path':
                result.successful = False
                result.reason = f"Parameter {param.name} cannot be changed runtime."
                self.get_logger().warn(result.reason)
            elif param.name == 'config_file_name':
                result.successful = False
                result.reason = f"Parameter {param.name} cannot be changed runtime."
                self.get_logger().warn(result.reason)
            elif param.name == 'number_of_messages_to_average_sample_time_over':
                result.successful = False
                result.reason = f"Parameter {param.name} cannot be changed runtime."
                self.get_logger().warn(result.reason)
            else:
                result.successful = False
                result.reason = f"Could not find {param.name} to be changed or parameter is not implemented for run-time change."
                self.get_logger().warn(result.reason)
        return result

def main(args=None):
    rclpy.init(args=args)
    filter = SignalFilter()
    rclpy.spin(filter)
    filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()