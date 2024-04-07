#ifndef B_LOCAL_PATH_PLANNER_HPP
#define B_LOCAL_PATH_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
//#include "tf2/convert.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <functional>
#include <memory>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include "roomba_500driver_meiji/msg/roomba_ctrl.hpp"

struct State
{
    double x;
    double y;
    double yaw;
    double vel;
    double yawrate;
};

struct Dynamic_Window
{
    double max_vel;
    double min_vel;
    double max_yawrate;
    double min_yawrate;
};

class DWAPlanner : public rclcpp::Node
{
    public:
        DWAPlanner();
        void process();
        void debager();

    private:
        void local_goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
        void obs_pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

        void roomba_ctl(double vel, double yawrate);
        void move_robot_image(State& state, double vel, double yawrate);
        void visualize_trajectory(const std::vector<State>& trajectory, const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& local_path_pub_, rclcpp::Time now);
        //void visualize_trajectories(const std::vector<std::vector<State>>& trajectory, const nav_msgs::msg::Path& local_path_pub,Time& now());
        double nomalize_angle(double angle);
        double calc_eval(const std::vector<State>& trajectory);
        double calc_heading_eval(const std::vector<State>& trajectory);
        double calc_velocity_eval(const std::vector<State>& trajectory);
        double calc_distance_eval(const std::vector<State>& trajectory);
        std::vector<State> calc_trajectory(double vel, double yawrate);

        bool is_goal_reached();
        void calc_dynamic_window();
        void broadcast_dynamic_tf();
        std::vector<double> calc_input();

        int hz_;
        double dt_;
        double goal_tolerance_;
        double max_vel_;
        double min_vel_;
        double max_yawrate_;
        double min_yawrate_;
        double max_accel_;
        double max_dyawrate_;
        double v_reso_;
        double y_reso_;
        double predict_time_;
        double heading_cost_gain_;
        double velocity_cost_gain_;
        double distance_cost_gain_;
        double robot_radius_;
        double radius_margin_;
        double search_range_;


        bool flag_local_goal_ = false;
        bool flag_obs_pose_ = false;

        //ros::NodeHandle nh_;
        //ros::NodeHandle private_nh_;

        State roomba_;
        Dynamic_Window dw_;

        //subscriber
        rclcpp::Subscription <geometry_msgs::msg::PointStamped>::SharedPtr local_goal_sub_; //目下で目指すべきゴール
        rclcpp::Subscription <geometry_msgs::msg::PoseArray>::SharedPtr obs_pose_sub_; //障害物位置
        
        //publisher
        rclcpp::Publisher<roomba_500driver_meiji::msg::RoombaCtrl>::SharedPtr cmd_speed_pub_; //roomba制御
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predict_path_pub_; //予想されるルート
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimal_path_pub_; //最適ルート
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;             
        

        geometry_msgs::msg::PointStamped local_goal_;
        geometry_msgs::msg::PoseArray obs_pose_;

        //std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
        //std::shared_ptr<geometry_msgs::msg::TransformStamped> transformStamped;
        //tf2_ros::TransformBroadcaster dynamic_br_;
        //tf2_ros::Buffer tfBuffer_;
        //tf2_ros::TransformListener tfListener_; 
        std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
        std::shared_ptr<tf2_ros::TransformListener> tfListener_;
        //std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
        //std::shared_ptr<tf2_ros::TransformListener> tfListener_;
        //tfBuffer_.setUsingDedicatedThread(true);


        roomba_500driver_meiji::msg::RoombaCtrl roomba_ctl_msg_;
        nav_msgs::msg::Path predict_path_;
        nav_msgs::msg::Path optimal_path_;
        nav_msgs::msg::Path local_path_;


        //rclcpp::Clock ros_clock(rcl_clock_type_t RCL_ROS_TIME);
        //rclcpp::TimerBase::SharedPtr timer_;
};

#endif // LOCAL_PATH_PLANNER_H

