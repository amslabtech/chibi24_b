#include "b_local_path_planner/b_local_path_planner.hpp"

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<DWAPlanner>();
	//rclcpp::Rate loop_rate(node->gettime());
    rclcpp::WallRate loop_rate(10);          
    //node->initialize();
	while (rclcpp::ok()) {
		node->process();
		rclcpp::spin_some(node);
	    loop_rate.sleep();
        RCLCPP_ERROR(node->get_logger(), "Error Message");
	}
	rclcpp::shutdown();
	return 0;
}