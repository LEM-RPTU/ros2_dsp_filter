#ifndef _CPP_PUBLISHER_H
#define _CPP_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"



class CppPublisher : public rclcpp::Node
{
    public:
        CppPublisher();
    private:
        // variables for node (ROS-related)
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist;
        OnSetParametersCallbackHandle::SharedPtr callback_handle;
        rclcpp::TimerBase::SharedPtr timer_pub_twist;
        // callback functions
        void publish_velocity_callback(); 
        // parameter callback
        rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters); 
        // attribute parameters
        struct parameter_set{
            bool publish_y_velocity_zero;
            std::string frame_id;
        };
        parameter_set params;
};

#endif
