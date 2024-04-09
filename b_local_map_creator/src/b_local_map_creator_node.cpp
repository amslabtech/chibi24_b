#include "b_local_map_creator/b_local_map_creator.hpp"
/*
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<LocalMapCreator> map_creator = nullptr;
  //map_creator = std::make_shared<LocalMapCreator>(node_options);
  map_creator = std::make_shared<LocalMapCreator>();
  // node_options.use_intra_process_comms(true);
  //auto node_ = rclcpp::Node::make_shared("b_local_map_creator_node");
  //std::shared_ptr<LocalMapCreator> b_local_map_creator = std::make_shared<LocalMapCreator>(node_options);
  rclcpp::spin(map_creator);
  rclcpp::shutdown();
  return 0;
}
*/

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<LocalMapCreator>();
	//rclcpp::Rate loop_rate(node->gettime());
  rclcpp::WallRate loop_rate(10);          
    //node->initialize();
	while (rclcpp::ok()) {
		node->process();
		rclcpp::spin_some(node);
	    loop_rate.sleep();
	}
	rclcpp::shutdown();
	return 0;
}


