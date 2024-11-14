#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult, Parameter
from filter_signal.parameters import ParameterSet
import filter_signal.filters as filters

class SignalFilter(Node):

    def __init__(self):
        super().__init__('signalfilter')
        self.init_parameters()
        self.sample_time_subscription = self.create_subscription(Float32, self.params.signal_topic, self.sample_time_subscription_callback_, 10)
        self.number_of_messages_received = 0

    def sample_time_subscription_callback_(self, msg: Float32):
        if(self.number_of_messages_received == 0):
            self.time_first_message_received = self.get_clock().now()
            self.number_of_messages_received += 1
            return
        elif(self.number_of_messages_received == self.params.number_of_messages_to_average_sample_time_over):
            self.sample_time = (self.get_clock().now() - self.time_first_message_received).nanoseconds / (1e9*self.number_of_messages_received)
            self.get_logger().info(f"Sample time is: {self.sample_time:.4f} seconds. Destroying sample-time subscription.")
            self.destroy_subscription(self.sample_time_subscription)
            self.low_pass_filter = filters.LowPassFilter(self.params.lowpass_cuttoff_frequency, self.sample_time)
            self.get_logger().info(f"Starting publisher of filtered message.")
            self.publisher_filtered_signal = self.create_publisher(Float32, self.params.signal_topic + '/filtered', 10)
            self.get_logger().info(f"Starting subscription for filtering.")
            self.subscription_sample_signal = self.create_subscription(Float32, self.params.signal_topic, self.subscription_sample_signal_callback_, 10)
            return
        else:
            self.number_of_messages_received += 1
            return
        
    def subscription_sample_signal_callback_(self, msg: Float32):
            filtered_signal = self.low_pass_filter.filter(msg.data)
            filtered_signal_msg = Float32()
            filtered_signal_msg.data = filtered_signal
            self.publisher_filtered_signal.publish(filtered_signal_msg)    

    def init_parameters(self):
        self.declare_parameter('signal_topic', 'sample_wave')
        self.declare_parameter('lowpass_cuttoff_frequency', 5.0) # Hz
        self.declare_parameter('number_of_messages_to_average_sample_time_over', 10) 
        # Read values of parameters
        self.params = ParameterSet(
            signal_topic = self.get_parameter('signal_topic').get_parameter_value().string_value,
            lowpass_cuttoff_frequency = self.get_parameter('lowpass_cuttoff_frequency').get_parameter_value().double_value,
            number_of_messages_to_average_sample_time_over = self.get_parameter('number_of_messages_to_average_sample_time_over').get_parameter_value().integer_value
        )
        
        self.add_on_set_parameters_callback(self.parametersCallback)

    def parametersCallback(self, parameters: list[Parameter]):
        result = SetParametersResult()
        result.successful = True
        for param in parameters:
            if param.name == 'signal_topic':
                result.successful = False
                result.reason = f"Parameter {param.name} cannot be changed runtime."
                self.get_logger().warn(result.reason)
            elif param.name == 'lowpass_cuttoff_frequency':
                result.successful = True
                result.reason = f"Parameter {param.name} changed successfully to {param.value:.2f}."
                self.params.lowpass_cuttoff_frequency = param.value.double_value
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

    # Destroy the node explicitly
    filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()