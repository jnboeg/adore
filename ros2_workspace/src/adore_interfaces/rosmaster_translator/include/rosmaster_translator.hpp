#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <numbers>
#include <cmath>

#include "adore_dynamics_adapters.hpp"
#include "adore_dynamics_conversions.hpp"
#include "adore_ros2_msgs/msg/vehicle_command.hpp"
#include "adore_ros2_msgs/msg/vehicle_state_dynamic.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"


using namespace std::chrono_literals;

const static float VEHICLE_COMMAND_DELTA_TIME_SECONDS = 0.05; // The main timer callback of trajectory tracker
const static float VEHICLE_WHEELBASE = 0.19;
//const static float ADORE_ROSMASTER_SCALE = 13.79;

namespace adore
{

    class RosmasterTranslator : public rclcpp::Node
    {
        private:
        std::unique_ptr<tf2_ros::Buffer>                tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener>     tf_listener;
        
        rclcpp::TimerBase::SharedPtr                    main_timer;

        /******************************* PUBLISHERS RELATED MEMBERS ************************************************************/                                 
        //Publisher to Rosmaster
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                     publisher_cmd_vel;
        //Publisher to ADORe
        rclcpp::Publisher<adore_ros2_msgs::msg::VehicleStateDynamic>::SharedPtr     publisher_vehicle_state_dynamic;

        /******************************* SUBSCRIBERS RELATED MEMBERS ************************************************************/
        rclcpp::Subscription<adore_ros2_msgs::msg::VehicleCommand>::SharedPtr   subscriber_vehicle_command;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr              subscriber_vel_raw;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr              subscriber_odom;

        /******************************* OTHER MEMBERS *************************************************************************/
        std::optional<dynamics::VehicleCommand> latest_vehicle_command = std::nullopt;
        adore_ros2_msgs::msg::VehicleStateDynamic last_vehicle_state_dynamic;
        geometry_msgs::msg::Twist latest_vel_raw;
        geometry_msgs::msg::Twist last_vel_raw;
        nav_msgs::msg::Odometry latest_odom;
        nav_msgs::msg::Odometry last_odom;
        geometry_msgs::msg::TransformStamped latest_tf;
        geometry_msgs::msg::TransformStamped initial_tf;

        rclcpp::Time current_time;
        rclcpp::Time last_update_time;

        bool initial_position_set = false;
        double initial_position_x = 0;
        double initial_position_y = 0;

        double offset_x = 0;
        double offset_y = 0;

        double start_position_x = 0;
        double start_position_y = 0;
        //double start_position_yaw = 0;

        int utm_zone;
        std::string utm_letter;

        double x, y;
        
        double roll, pitch, yaw;
        double yaw_rate = 0.0;

        double velocity = 0.0; // This assumption is fine when the vehicle starts standing
        double steering_rate = 0.0;

        double acceleration = 0.0;
        double steering_angle = 0.0;

        public:
        explicit RosmasterTranslator();

        void timer_callback();

        void load_parameters();

        void calculate_offset(const double& x, const double& y);
        void get_tf_transform();
        void calculate_yaw(geometry_msgs::msg::Quaternion quarternion);

        double calculate_yaw_rate(const double& velocity, const double& steering_angle);

        double euler_integrate_acceleration( const double& acceleration );
        double deriviate_steering_angle( const double& steering_angle_latest, const double& steering_angle_last);

        double deriviate_yaw_rate(const double& yaw_latest, const double& yaw_last);
        double deriviate_velocity(const double& velocity_latest, const double& velocity_last);
        double euler_integrate_steering_rate(const double& steering_rate);
    };
}
