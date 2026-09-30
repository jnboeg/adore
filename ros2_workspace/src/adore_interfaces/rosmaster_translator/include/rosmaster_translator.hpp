#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "adore_dynamics_adapters.hpp"
#include "adore_dynamics_conversions.hpp"
#include "adore_ros2_msgs/msg/vehicle_command.hpp"
#include "adore_ros2_msgs/msg/vehicle_state_dynamic.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"


using namespace std::chrono_literals;

const static float VEHICLE_COMMAND_DELTA_TIME_SECONDS = 0.05; // The main timer callback of trajectory tracker

const static float ADORE_ROSMASTER_SCALE = 13.79;

namespace adore
{

    class RosmasterTranslator : public rclcpp::Node
    {
        private:
        std::unique_ptr<tf2_ros::Buffer>                tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener>     tf_listener;
        
        /******************************* PUBLISHERS RELATED MEMBERS ************************************************************/
        rclcpp::TimerBase::SharedPtr                                                main_timer;                                 
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                     publisher_cmd_vel;
        rclcpp::Publisher<adore_ros2_msgs::msg::VehicleStateDynamic>::SharedPtr     publisher_vehicle_state_dynamic;

        /******************************* SUBSCRIBERS RELATED MEMBERS ************************************************************/
        rclcpp::Subscription<adore_ros2_msgs::msg::VehicleCommand>::SharedPtr   subscriber_vehicle_command;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr              subscriber_vel_raw;

        /******************************* OTHER MEMBERS *************************************************************************/
        std::optional<dynamics::VehicleCommand> latest_vehicle_command = std::nullopt;
        geometry_msgs::msg::Twist latest_vel_raw;
        geometry_msgs::msg::Twist last_vel_raw;
        geometry_msgs::msg::TransformStamped latest_tf;
        geometry_msgs::msg::TransformStamped initial_tf;

        double x, y;

        double roll, pitch, yaw;

        double velocity = 0.0; // This assumption is fine when the vehicle starts standing
        double steering_rate = 0.0;

        double acceleration = 0.0;
        double steering_angle = 0.0;

        public:
        explicit RosmasterTranslator();

        void timer_callback();

        double euler_integrate_acceleration( const double& acceleration );
        double deriviate_steering_angle( const double& steering_angle);

        double deriviate_velocity(const double& velocity_latest, const double& velocity_last);
        double euler_integrate_steering_rate(const double& steering_rate);
    };
}
