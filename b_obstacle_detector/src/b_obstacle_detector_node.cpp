#include "b_obstacle_detector/b_obstacle_detector.hpp"

int main(int argc, char *argv[])
{
  std::cout << __LINE__ << std::endl;
  printf("main1\n");
  rclcpp::init(argc,argv);
  
  printf("main2\n");
  // ノードを生成する
  auto node_ = rclcpp::Node::make_shared("b_obstacle_detector_node");
  std::shared_ptr<ObstacleDetector> odetector = std::make_shared<ObstacleDetector>();
  rclcpp::spin(odetector);
  rclcpp::shutdown();

  return 0;
}
