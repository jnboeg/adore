#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "adore_dynamics_adapters.hpp"
#include "adore_dynamics_conversions.hpp"
#include "adore_ros2_msgs/msg/vehicle_command.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"


using namespace std::chrono_literals;

const static float VEHICLE_COMMAND_DELTA_TIME_SECONDS = 0.05; // The main timer callback of trajectory tracker

const static float ADORE_ROSMASTER_SCALE = 13.79;

namespace adore
{

    class RosmasterTranslator : public rclcpp::Node
    {
        private:
        /******************************* PUBLISHERS RELATED MEMBERS ************************************************************/
        rclcpp::TimerBase::SharedPtr                                main_timer;                                 
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr     publisher_cmd_vel;

        /******************************* SUBSCRIBERS RELATED MEMBERS ************************************************************/
        rclcpp::Subscription<adore_ros2_msgs::msg::VehicleCommand>::SharedPtr      subscriber_vehicle_command;

        /******************************* OTHER MEMBERS *************************************************************************/
        std::optional<dynamics::VehicleCommand> latest_vehicle_command = std::nullopt;

        double velocity = 0.0; // This assumption is fine when the vehicle starts standing

        double steering_rate = 0.0;

        public:
        explicit RosmasterTranslator();

        void timer_callback();
        
        void vehicle_command_callback(const dynamics::VehicleCommand& msg);

        double euler_integrate_acceleration( const double& acceleration );
        double velocity_scaling(const double& acceleration);
        double deriviate_steering_angle( const double& steering_angle);
    };
}
