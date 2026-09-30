/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include "rosmaster_translator.hpp"
#include <dynamics/vehicle_command.hpp>

using namespace std::chrono_literals;

namespace adore
{
    RosmasterTranslator::RosmasterTranslator() : Node("rosmaster_translator")
    {
		tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    	tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

		main_timer = create_wall_timer(50ms, std::bind(&RosmasterTranslator::timer_callback, this));
		publisher_cmd_vel = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
		//publisher_vehicle_state_dynamic = create_publisher<adore_ros2_msgs::msg::VehicleStateDynamic>("/ego_vehicle_state_dynamic", 1);

		subscriber_vehicle_command = create_subscription<adore_ros2_msgs::msg::VehicleCommand>( "/ego_vehicle/next_vehicle_command", 1,
                                      		[this](const adore_ros2_msgs::msg::VehicleCommand& msg) { latest_vehicle_command = dynamics::conversions::to_cpp_type(msg); });
		subscriber_vel_raw = create_subscription<geometry_msgs::msg::Twist>("/vel_raw", 1, 
									[this](const geometry_msgs::msg::Twist& msg) { latest_vel_raw = msg; });

	}

    /******************************* PUBLISHER RELATED FUNCTIONS ************************************************************/


    void RosmasterTranslator::timer_callback()
    {
    	if (!this->latest_vehicle_command.has_value())
    	{
    		return;
    	}

    	dynamics::VehicleCommand vehicle_command = this->latest_vehicle_command.value();

  		geometry_msgs::msg::Twist message;
  		//message = "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}";
		message.linear.x = euler_integrate_acceleration(vehicle_command.acceleration);
  		message.linear.y = 0.0;
  		message.linear.z = 0.0;
  		message.angular.x = 0.0;
  		message.angular.y = 0.0;
		message.angular.z = deriviate_steering_angle(vehicle_command.steering_angle);

		geometry_msgs::msg::Twist vel_raw;
		this->last_vel_raw = vel_raw;
		vel_raw = this->latest_vel_raw;

		auto transform = tf_buffer->lookupTransform(
			"map",
			"base_link",
			tf2::TimePointZero
		);

		x = transform.transform.translation.x;
		y = transform.transform.translation.y;

		tf2::Quaternion quaternion(transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
		
		tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

		adore_ros2_msgs::msg::VehicleStateDynamic state;
		state.x = x;
		state.y = y;
		state.z = 0.0;
		state.vx = vel_raw.linear.x;
		state.vy = 0.0;
		state.yaw_angle = yaw;
		//state.yaw_rate = ;
		state.steering_angle = euler_integrate_steering_rate(vel_raw.angular.z);
		state.steering_rate = vel_raw.angular.z;
		state.ax = deriviate_velocity(vel_raw.linear.x, last_vel_raw.linear.x);
		state.ay = 0.0;
		//state.frame_id = "base_link"; ???

  		RCLCPP_INFO(this->get_logger(), "Velocity: '%f', Steering rate: '%f'", message.linear.x, message.angular.z);
  		publisher_cmd_vel->publish(message);
    }

    double RosmasterTranslator::euler_integrate_acceleration( const double& acceleration )
    {
      velocity += acceleration * VEHICLE_COMMAND_DELTA_TIME_SECONDS; 

      if ( velocity < 0.0 )
      {
        velocity = 0.0;
      }
      
      return velocity;
    }

	double RosmasterTranslator::deriviate_steering_angle( const double& steering_angle)
	{
		steering_rate = steering_angle / VEHICLE_COMMAND_DELTA_TIME_SECONDS;

		return steering_rate;
	}

	double RosmasterTranslator::deriviate_velocity( const double& velocity_latest, const double& velocity_last) {
		acceleration = (velocity_latest - velocity_last) / VEHICLE_COMMAND_DELTA_TIME_SECONDS;

		return acceleration;
	}

	double RosmasterTranslator::euler_integrate_steering_rate(const double& steering_rate) {
		steering_angle += steering_rate * VEHICLE_COMMAND_DELTA_TIME_SECONDS;

		return steering_angle;
	}
}
