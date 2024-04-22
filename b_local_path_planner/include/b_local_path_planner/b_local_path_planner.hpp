#ifndef B_LOCAL_PATH_PLANNER_HPP
#define B_LOCAL_PATH_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
//#include <tf2/transform_datatypes.h>//
//#include <tf2/LinearMath/Transform.h>//
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <functional>
#include <memory>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
//#include <geometry_msgs/msg/pose.hpp> //
#include <tf2_ros/transform_listener.h>
//#include <tf2_ros/transform_broadcaster.h> //
//#include <tf2/LinearMath/Quaternion.h>//
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
        void change_mode();
        //void broadcast_dynamic_tf();
        std::vector<double> calc_input();

        //double weight_heading_;
        //double weight_vel_;
        //double weight_dist_;
        //double robot_radius_;

        // ----- 変数 -----
        bool   is_visible_;         // パスを可視化するかの設定用
        int    hz_ = 10;                 // ループ周波数
        int    mode_;
        double max_vel_;            // 最高並進速度 [m/s]（計算用）
        double max_vel1_;           // 最高並進速度 [m/s]（平常時）
        double max_vel2_;           // 最高並進速度 [m/s]（減速時）
        double turn_thres_yawrate_; // 旋回中か判断する閾値
        double avoid_thres_vel_;    // 回避中か判断する閾値
        double mode_log_time_;      // logをとる時間区間
        double min_vel_;            // 最低並進速度 [m/s]
        double max_yawrate_;        // 最高旋回速度 [rad/s]（計算用）
        double max_yawrate1_;       // 最高旋回速度 [rad/s]（平常時）
        double max_yawrate2_;       // 最高旋回速度 [rad/s]（減速時）
        double max_accel_;          // 最高並進加速度 [m/s^2]
        double max_dyawrate_;       // 最高旋回加速度 [rad/s^2]
        double vel_reso_;           // 並進速度解像度 [m/s]
        double yawrate_reso_;       // 旋回速度解像度 [rad/s]
        double stop_vel_th_;        // 停止状態か判断する閾値 [m/s^2]
        double stop_yawrate_th_;    // 停止状態か判断する閾値 [rad/s]
        double dt_;                 // 微小時間 [s]
        double predict_time_;       // 軌跡予測時間 [s]
        double predict_time1_;      // 軌跡予測時間 [s]
        double predict_time2_;      // 軌跡予測時間 [s]
        double roomba_radius_;      // Roombaのサイズ(半径) [m]
        double radius_margin_;      // 半径の余白 [m]（計算用）
        double radius_margin1_;     // 半径の余白 [m]（平常時）
        double radius_margin2_;     // 半径の余白 [m]（減速時）
        double goal_tolerance_;     // 目標地点の許容誤差 [m]
        double search_range_;       // 評価関数distで探索する範囲 [m]

        


        bool flag_local_goal_ = false;
        bool flag_obs_pose_ = false;

        // mode記録用
        std::vector<double> mode_log_;


        // 重み定数
        double weight_heading_;  //（計算用）
        double weight_heading1_; //（平常時）
        double weight_heading2_; //（減速時）
        double weight_dist_;     //（計算用）
        double weight_dist1_;    //（平常時）
        double weight_dist2_;    //（減速時）
        double weight_vel_;
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

        geometry_msgs::msg::PointStamped local_goal_;
        geometry_msgs::msg::PoseArray obs_pose_;

        //tf2_ros::Buffer tfBuffer_;
        //tf2_ros::TransformListener tfListener_; 
        //std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
        //std::shared_ptr<tf2_ros::TransformListener> tfListener_;
        std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
        std::shared_ptr<tf2_ros::TransformListener> tfListener_;
        //tfBuffer_.setUsingDedicatedThread(true);


        roomba_500driver_meiji::msg::RoombaCtrl roomba_ctl_msg_;
        nav_msgs::msg::Path predict_path_;
        nav_msgs::msg::Path optimal_path_;

        //rclcpp::Clock ros_clock(rcl_clock_type_t RCL_ROS_TIME);
        //rclcpp::TimerBase::SharedPtr timer_;
};

#endif // LOCAL_PATH_PLANNER_H

