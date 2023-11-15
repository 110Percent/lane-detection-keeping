#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "lane_interfaces/msg/movement_instruction.hpp"

using std::placeholders::_1;

/**
 *	Class responsible for receivimg abstract movement commands and converting them to hardware commands, then sending those to the hardware.
 */
class MovementController : public rclcpp::Node
{
	public:
		MovementController()
			: Node("movement_controller")
		{
			// Create the subscription to the movement_instructions topic.
			subscription_ = this->create_subscription<lane_interfaces::msg::MovementInstruction>(
					"movement_instructions", 10, std::bind(&MovementController::movement_instruction_callback, this, _1));
		}
		
	private:
		// Code to execute when receiving a movement command
		void movement_instruction_callback(const lane_interfaces::msg::MovementInstruction::SharedPtr msg) {
			// For now, just echo the string received.
			RCLCPP_INFO(this->get_logger(), "Received message: %s\n---------------", msg->temp.c_str());
		}

		rclcpp::Subscription<lane_interfaces::msg::MovementInstruction>::SharedPtr subscription_;	
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MovementController>());
	rclcpp::shutdown();
	return 0;
}
