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
		main_timer = create_wall_timer(50ms, std::bind(&RosmasterTranslator::timer_callback, this));
		publisher_cmd_vel = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

		subscriber_vehicle_command = create_subscription<adore_ros2_msgs::msg::VehicleCommand>( "/ego_vehicle/next_vehicle_command", 1,
                                      		[this](const adore_ros2_msgs::msg::VehicleCommand& msg) { latest_vehicle_command = dynamics::conversions::to_cpp_type(msg); });
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
  		message.linear.x = velocity_scaling(euler_integrate_acceleration(vehicle_command.acceleration));
		//message.linear.x = euler_integrate_acceleration(vehicle_command.acceleration);
  		message.linear.y = 0.0;
  		message.linear.z = 0.0;
  		message.angular.x = 0.0;
  		message.angular.y = 0.0;
		message.angular.z = deriviate_steering_angle(vehicle_command.steering_angle);
  		//message.angular.z = 0.5;

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

	double RosmasterTranslator::velocity_scaling(const double& velocity)
	{
		return velocity / ADORE_ROSMASTER_SCALE;
	}

	double RosmasterTranslator::deriviate_steering_angle( const double& steering_angle)
	{
		steering_rate = steering_angle / VEHICLE_COMMAND_DELTA_TIME_SECONDS;

		return steering_rate;
	}
}
