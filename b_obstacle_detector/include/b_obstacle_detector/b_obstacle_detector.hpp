#ifndef b_obstacle_detector_HPP
#define b_obstacle_detector_HPP

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <optional>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <math.h>

using namespace std::chrono_literals;

class ObstacleDetector : public rclcpp::Node
{
  public:
  ObstacleDetector(); 
  void process();

  private:
  //コールバック関数
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg); 
  void timer_callback();

  //関数
  //void set_obstacle_detecter();
  void scan_obstacle();
  bool is_ignore_scan(double angle);


  //変数
  int hz_;
  int laser_step_;
  double ignore_dist_;

  std::string robot_frame_;
  std::vector<double> ignore_angle_range_list_;
  bool flag_laser_scan_ = false;

  std::optional<sensor_msgs::msg::LaserScan> scan_;
  //obstacle_detecter
  geometry_msgs::msg::PoseArray obstacle_pose_array_;


  rclcpp::Subscription <sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_pose_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // b_obstacle_detector_hpp