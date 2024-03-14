#include "b_obstacle_detector/b_obstacle_detector.hpp"

int main(int argc, char *argv[])
{
  printf("main1\n");
  rclcpp::init(argc,argv);
  // yaml_パラメータ設定
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  // node_options.use_intra_process_comms(true);

  //std::shared_ptr<ObstacleDetector> b_obstacle_detector = nullptr;
  //b_obstacle_detector = std::make_shared<ObstacleDetector>("b_obstacle_detector_topic", "b_obstacle_detector", node_options);
  auto b_obstacle_detector = std::make_shared<ObstacleDetector>("b_obstacle_detector_topic", "b_obstacle_detector", node_options);
  
  printf("main2\n");
  //std::shared_ptr<ObstacleDetector> schallenge = std::make_shared<ObstacleDetector>("b_obstacle_detector_topic", "b_obstacle_detector", node_options);
  rclcpp::spin(b_obstacle_detector);
  rclcpp::shutdown();

  return 0;
}
