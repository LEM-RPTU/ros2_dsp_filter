#include "cpp_python_package/cpp_publisher.hpp"

CppPublisher::CppPublisher() : Node("cpp_publisher"){
    this->declare_parameter("cmd_vel_topic", "cmd_vel");
    this->declare_parameter("frame_id", "base_link");
    this->declare_parameter("publish_y_velocity_zero", false);
    // Publishers
    pub_twist = this->create_publisher<geometry_msgs::msg::TwistStamped>(this->get_parameter("cmd_vel_topic").as_string(), 1);
    callback_handle = this->add_on_set_parameters_callback(std::bind(&CppPublisher::parameters_callback, this, std::placeholders::_1));
    timer_pub_twist = this->create_wall_timer(std::chrono::milliseconds(100), [this](){this->publish_velocity_callback();});
    //Initialize Parameters
    this->params.frame_id =  this->get_parameter("frame_id").as_string();
    this->params.publish_y_velocity_zero = this->get_parameter("publish_y_velocity_zero").as_bool();
    // 
    RCLCPP_INFO(this->get_logger(), "Initialization is done.");
}

void CppPublisher::publish_velocity_callback(){
    geometry_msgs::msg::TwistStamped msg = geometry_msgs::msg::TwistStamped();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = this->params.frame_id;
    msg.twist.linear.x = 0.5;
    msg.twist.angular.z = 0.1;
    if(this->params.publish_y_velocity_zero){ //demonstrating parameter changes at runtime
        msg.twist.linear.y = 0.0;
    }else{
        msg.twist.linear.y = 1.0;
    }
    pub_twist->publish(msg);
    RCLCPP_DEBUG(this->get_logger(), "Published a new twist message.");
}

rcl_interfaces::msg::SetParametersResult CppPublisher::parameters_callback(const std::vector<rclcpp::Parameter> &parameters){
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "";
    // Set class attributes
    for (const rclcpp::Parameter &parameter : parameters){
        if (parameter.get_name() == "publish_y_velocity_zero"){
            this->params.publish_y_velocity_zero = parameter.as_bool();
            std::string success_message = "Parameter " + parameter.get_name() + " was changed.";
            RCLCPP_INFO(this->get_logger(), success_message.c_str());
            result.reason = result.reason + std::string("\n") + success_message;
        }else if(parameter.get_name() == "frame_id"){
            result.successful = false;
            std::string failure_message = "Parameter " + parameter.get_name() + " change had not effect, as it is not advisable to change it at runtime.";
            RCLCPP_WARN(this->get_logger(), failure_message.c_str());
            result.reason = result.reason + std::string("\n") + failure_message;
        }else{// should never happen, because ROS catches non-declared parameters
            result.successful = false;
            std::string failure_message = "Parameter " + parameter.get_name() + " change was not successful. It is either not implemented or the parameter does not exist.";
            RCLCPP_WARN(this->get_logger(), failure_message.c_str());
            result.reason = result.reason + std::string("\n") +  failure_message;
        }
    }
    // configure answer
    return result;
}
