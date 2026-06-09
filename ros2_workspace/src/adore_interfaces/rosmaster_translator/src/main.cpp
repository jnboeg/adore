#include "rclcpp/rclcpp.hpp"
#include "rosmaster_translator.hpp"

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<adore::RosmasterTranslator>());
	rclcpp::shutdown();
	return 0;
}
