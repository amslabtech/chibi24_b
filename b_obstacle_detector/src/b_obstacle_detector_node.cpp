#include "b_obstacle_detector/b_obstacle_detector.hpp"
/*
int main(int argc, char *argv[])
{
  rclcpp::init(argc,argv);
  // ノードを生成する
  //auto node_ = rclcpp::Node::make_shared("b_obstacle_detector");
  std::shared_ptr<ObstacleDetector> odetector = std::make_shared<ObstacleDetector>();
  rclcpp::spin(odetector);
  rclcpp::shutdown();

  return 0;
}
*/

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ObstacleDetector>();
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
