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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class RosmasterTranslator : public rclcpp::Node
{
  private:
    /******************************* PUBLISHERS RELATED MEMBERS ************************************************************/
    rclcpp::TimerBase::SharedPtr mainTimer;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisherString;

    /******************************* SUBSCRIBERS RELATED MEMBERS ************************************************************/
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriberString;

    /******************************* OTHER MEMBERS *************************************************************************/
    std_msgs::msg::String latestRecievedStringMessage;

  public:
    RosmasterTranslator() : Node("rosmaster_translator")
    {
      mainTimer = this->create_wall_timer(100ms, std::bind(&RosmasterTranslator::Run, this));
      publisherString = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

      subscriberString = this->create_subscription<std_msgs::msg::String>("subscribing_topic_name", 10, std::bind(&RosmasterTranslator::SubscriberStringCallback, this, std::placeholders::_1));
    }

    /******************************* PUBLISHER RELATED FUNCTIONS ************************************************************/

    void Run(){
      geometry_msgs::msg::Twist message;
      //message = "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}";
      message.linear.x = 0.1;
      message.linear.y = 0.0;
      message.linear.z = 0.0;
      message.angular.x = 0.0;
      message.angular.y = 0.0;
      message.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Velocity: '%f'", message.linear.x);
      publisherString->publish(message);
      rclcpp::sleep_for(std::chrono::seconds(1));
    }

    /******************************* SUBSCRIBER RELATED FUNCTIONS************************************************************/

    void SubscriberStringCallback(std_msgs::msg::String msg){
      latestRecievedStringMessage = msg;
    }
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RosmasterTranslator>());
  rclcpp::shutdown();
  return 0;
}
