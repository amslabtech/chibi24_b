#include "b_local_goal_creator/b_local_goal_creator.hpp"

LocalGoalCreator::LocalGoalCreator() : Node("LocalGoalCreator")
{
    this  -> declare_parameter("hz", 10);
    this  -> declare_parameter("index_step", 5);
    this  -> declare_parameter("target_distance", 5.0);
    this  -> declare_parameter("goal_index", 50);

    this  -> get_parameter("hz", hz_);
    this  -> get_parameter("index_step", index_step_);
    this  -> get_parameter("target_distance", taeget_distance_);
    this  -> get_parameter("goal_index", goal_index_);

    goal_.header.frame_id = "map";

    path_sub_   = this->create_subscription<nav_msgs::msg::Path>(
    "/global_path", rclcpp::QoS(1).reliable(),
    std::bind(&LocalGoalCreator::pathCallback, this, std::placeholders::_1));

    pose_sub_   = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/estimated_pose", rclcpp::QoS(1).reliable(),
    std::bind(&LocalGoalCreator::poseCallback, this, std::placeholders::_1));

    local_goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "/local_goal", rclcpp::QoS(1).reliable());

    //path_sub_ = nh_.subscribe("/global_path", 1, &LocalGoalCreator::pathCallback, this);
    //pose_sub_ = nh_.subscribe("/estimate_pose", 1, &LocalGoalCreator::poseCallback, this);
    //local_goal_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/local_goal", 1);
}

void LocalGoalCreator::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    pose_ = *msg;
    //printf("poseCallback\n");
    process();
}

void LocalGoalCreator::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    path_ = msg->poses;
    is_path_ = true;
    //printf("pathCallback\n");
    
}
int LocalGoalCreator::getOdomFreq() { return hz_; }

void LocalGoalCreator::process()
{


    if(is_path_)
    {
        publishGoal();
    }
    else{
        //printf("error\n");
    }

}

void LocalGoalCreator::publishGoal()
{
    double distance = getDistance();
    while(distance < taeget_distance_)
    {
        goal_index_ += index_step_;
        distance = getDistance();
        if(goal_index_ >= path_.size())
        {
            goal_index_ = path_.size() - 1;
            break;
        }
    }
    
    goal_.point = path_[goal_index_].pose.position;
    //printf("pub\n");
    goal_.header.stamp = this->get_clock()->now();
    local_goal_pub_->publish(goal_);
}

double LocalGoalCreator::getDistance()
{
    double dx = path_[goal_index_].pose.position.x - pose_.pose.position.x;
    double dy = path_[goal_index_].pose.position.y - pose_.pose.position.y;
    return hypot(dx, dy);
}

