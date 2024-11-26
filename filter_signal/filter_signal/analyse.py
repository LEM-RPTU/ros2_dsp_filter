#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import ros2topic.api
import yaml
import os
from filter_signal.analyse_parameters import ParameterSet
from rcl_interfaces.msg import SetParametersResult, Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class GenericFilter(Node):

    def __init__(self):
        super().__init__('topic_analyser')
        self.init_parameters()
        #namespaced_topic_name = self.get_namespace() + '/' + self.params.topic
        message_type = ros2topic.api.get_msg_class(self, self.params.topic, blocking=True, include_hidden_topics=True)
        #QoS Settings should be compatible with whatever the publisher is doing, see:
        # https://docs.ros.org/en/iron/Concepts/Intermediate/About-Quality-of-Service-Settings.html#quality-of-service-settings
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        self.subscription = self.create_subscription(message_type, self.params.topic, self.subscription_callback, qos_profile=qos_profile)
        

    def subscription_callback(self, msg):
        message_yaml = yaml.safe_load(ros2topic.api.message_to_yaml(msg))
        result_full_path = os.path.join(self.params.config_path, self.params.result_file_name + '.yaml')
        try:
            with open(result_full_path, 'w') as file:
                yaml.dump(message_yaml, file)
            self.get_logger().info('Please edit and use the following yaml in the config path: {} to filter the message'.format(result_full_path))
        except FileNotFoundError:
            self.get_logger().error('Could not write to file: {}. Check path validity and change parameters if necessary.'.format(result_full_path))

    def init_parameters(self):
        self.declare_parameter('topic', 'imu') 
        self.declare_parameter('config_path', os.path.expanduser("~"))
        self.declare_parameter('result_file_name', 'analytic_result')
        self.params = ParameterSet(
            topic = self.get_parameter('topic').get_parameter_value().string_value,
            config_path = self.get_parameter('config_path').get_parameter_value().string_value,
            result_file_name = self.get_parameter('result_file_name').get_parameter_value().string_value
        )
        for parameter in self.params.__dict__.keys():
            self.get_logger().info(f"Parameter {parameter} is initialized as: {self.params.__dict__[parameter]}")
        self.add_on_set_parameters_callback(self.parametersCallback)

    def parametersCallback(self, parameters: list[Parameter]):
        result = SetParametersResult()
        result.successful = True
        for param in parameters:
            if param.name == 'topic':
                result.successful = False
                result.reason = f"Parameter {param.name} cannot be changed runtime."
                self.get_logger().warn(result.reason)
            elif param.name == 'config_path':
                result.successful = False
                result.reason = f"Parameter {param.name} cannot be changed runtime."
                self.get_logger().info(result.reason)
            else:
                result.successful = False
                result.reason = f"Could not find {param.name} to be changed or parameter is not implemented for run-time change."
                self.get_logger().warn(result.reason)
        return result


def main(args=None):
    rclpy.init(args=args)
    filter = GenericFilter()
    rclpy.spin_once(filter, timeout_sec=10.0)
    filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()