#ifndef b_local_map_creator_hpp
#define b_local_map_creator_hpp

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <optional>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

using namespace std::chrono_literals;

class LocalMapCreator : public rclcpp::Node
{
    public:
    //LocalMapCreator(const rclcpp::NodeOptions & node_options);
    LocalMapCreator();

    void process();

    private:
    //コールバック関数
    void obs_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void timer_callback();

    //関数
    bool is_in_local_map(const double x, const double y);
    int get_grid_index(const double dist, const double amgle);
    int xy_to_index(const double x, const double y);
    void init_local_map();
    void update_local_map();
    void init_map();

    //変数
    int hz_;
    double local_map_size_;
    double local_map_resolution_;


    bool is_get_obs_poses_ = false;

    nav_msgs::msg::OccupancyGrid local_map_;
    geometry_msgs::msg::PoseArray obs_poses_;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_obs_poses_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_local_map_;

    rclcpp::TimerBase::SharedPtr timer_;
};

#endif