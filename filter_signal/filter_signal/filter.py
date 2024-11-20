#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult, Parameter
from filter_signal.filter_parameters import ParameterSet
from scipy import signal

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
            # calulate sample time & then destroy the subscription
            self.sample_time = (self.get_clock().now() - self.time_first_message_received).nanoseconds / (1e9*self.number_of_messages_received)
            self.get_logger().debug(f"Destroying sample-time subscription.")
            self.destroy_subscription(self.sample_time_subscription)
            self.sample_frequency = (1.0/self.sample_time)
            self.get_logger().info(f"Sample time is: {self.sample_time:.4f} s.")
            self.get_logger().info(f"Sample frequency is: {self.sample_frequency:.4f} Hz.")
            # Initialize filters
            self.init_filters()
            # start subscribing signal and publishing filtered signal
            self.get_logger().debug(f"Starting publisher of filtered message.")
            self.publisher_filtered_signal = self.create_publisher(Float32, self.params.signal_topic + '/filtered', 10)
            self.get_logger().debug(f"Starting subscription for filtering.")
            self.subscription_sample_signal = self.create_subscription(Float32, self.params.signal_topic, self.subscription_sample_signal_callback_, 10)
            return
        else:
            self.number_of_messages_received += 1
            return
        
    def subscription_sample_signal_callback_(self, msg: Float32):
            filtered_signal, self.filter_state = signal.lfilter(self.b, self.a, [msg.data], zi=self.filter_state)
            filtered_signal_msg = Float32()
            filtered_signal_msg.data = filtered_signal[0]
            self.publisher_filtered_signal.publish(filtered_signal_msg)
        
    def init_filters(self):
            if self.params.filter_type == 'lowpass':
                self.b, self.a = signal.butter(N=self.params.filter_order, 
                                               Wn=self.params.frequency_band[0], 
                                               btype=self.params.filter_type, 
                                               analog=False, 
                                               fs=self.sample_frequency)
            elif self.params.filter_type == 'highpass':
                self.b, self.a = signal.butter(N=self.params.filter_order, 
                                               Wn=self.params.frequency_band[1], 
                                               btype=self.params.filter_type, 
                                               analog=False, 
                                               fs=self.sample_frequency)
            else:
                self.b, self.a = signal.butter(N=self.params.filter_order, 
                                               Wn=self.params.frequency_band, 
                                               btype=self.params.filter_type, 
                                               analog=False, 
                                               fs=self.sample_frequency)
            self.filter_state = signal.lfilter_zi(self.b, self.a)

    def init_parameters(self):
        self.declare_parameter('signal_topic', 'sample_wave')
        self.declare_parameter('frequency_band', [1.0, 50.0]) # Hz, must be strictly positive
        self.declare_parameter('number_of_messages_to_average_sample_time_over', 100) #min. 1 and integer
        self.declare_parameter('filter_type', 'lowpass') # {lowpass, highpass, bandpass, bandstop} are supported
        self.declare_parameter('filter_order', 4) # min. 1 and integer
        # Read values of parameters
        self.params = ParameterSet(
            signal_topic = self.get_parameter('signal_topic').get_parameter_value().string_value,
            frequency_band = self.get_parameter('frequency_band').get_parameter_value().double_array_value,
            number_of_messages_to_average_sample_time_over = self.get_parameter('number_of_messages_to_average_sample_time_over').get_parameter_value().integer_value,
            filter_type = self.get_parameter('filter_type').get_parameter_value().string_value,
            filter_order = self.get_parameter('filter_order').get_parameter_value().integer_value
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
            elif param.name == 'frequency_band':
                result.successful = False
                result.reason = f"Parameter {param.name} cannot be changed runtime."
                self.get_logger().warn(result.reason)
            elif param.name == 'number_of_messages_to_average_sample_time_over':
                result.successful = False
                result.reason = f"Parameter {param.name} cannot be changed runtime."
                self.get_logger().warn(result.reason)
            elif param.name == 'filter_type':
                result.successful = False
                result.reason = f"Parameter {param.name} cannot be changed runtime."
                self.get_logger().warn(result.reason)
            elif param.name == 'filter_order':
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