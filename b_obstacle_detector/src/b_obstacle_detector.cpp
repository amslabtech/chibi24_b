#include "b_obstacle_detector/b_obstacle_detector.hpp"

ObstacleDetector::ObstacleDetector()
: Node("b_obstacle_detector")
{
  std::cout << __LINE__ << std::endl;
  timer_ = this->create_wall_timer(0.5s, std::bind(&ObstacleDetector::timer_callback, this));
  //auto hz_ = this->get_parameter("hz").as_int();
  std::cout << __LINE__ << std::endl;
  ignore_dist_ = this->declare_parameter<double>("ignore_dist",3);
  std::cout << __LINE__ << std::endl;
  laser_step_ = this->declare_parameter<int>("laser_step",0.01); 
  std::cout << __LINE__ << std::endl;
  ignore_angle_range_list_ = this->declare_parameter<std::vector<double>>("ignore_angle_range_list",{(3.0*M_PI/16.0), (5.0*M_PI/16.0), (11.0*M_PI/16.0)});
  std::cout << __LINE__ << std::endl;

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",rclcpp::QoS(1).reliable(),std::bind(&ObstacleDetector::scan_callback,this,std::placeholders::_1));
  obstacle_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/obstacle_pose",rclcpp::QoS(1).reliable());

  obstacle_pose_array_.header.frame_id = "base_link";
}

void ObstacleDetector::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // 一定周期で行う処理を書く
    scan_ = *msg;
    flag_laser_scan_ = true;
}

void ObstacleDetector::timer_callback()
{
    // 一定周期で行う処理を書く
    process();
}


void ObstacleDetector::process()
{
    printf("process");
    //ros::Rate loop_rate(hz_);

    //nodeが続く間繰り返される
    while(rclcpp::ok())
    {
        if(flag_laser_scan_)
        {
            scan_obstacle();
        }
        auto node_ = rclcpp::Node::make_shared("b_obstacle_detector_node");
        rclcpp::spin_some(node_);
        rclcpp::WallRate loop_rate(500ms);
        loop_rate.sleep();
    }
}

//obstacle pose.position x,yにlidar 情報挿入
void ObstacleDetector::scan_obstacle()
{
    obstacle_pose_array_.poses.clear();

    for(int i=0;i<scan_.value().ranges.size();i+=laser_step_)
    {
        const double angle = scan_.value().angle_min + scan_.value().angle_increment * i;
        //障害物からlidarまでの距離
        const double range = scan_.value().ranges[i];

        if(is_ignore_scan(angle))
        {
            continue;
        }

        if(range < ignore_dist_)
        {
            continue;
        }

        geometry_msgs::msg::Pose obs_pose;
        obs_pose.position.x = range * cos(angle);
        obs_pose.position.y = range * sin(angle);
        obstacle_pose_array_.poses.push_back(obs_pose);
    }
    obstacle_pose_pub_->publish(obstacle_pose_array_);
}


//無視するlidarの範囲の決定
bool ObstacleDetector::is_ignore_scan(double angle)
{
    angle = abs(angle);
    const int size = ignore_angle_range_list_.size();

    for(int i=0; i<size/2; i++)
    {
        if(ignore_angle_range_list_[i*2] < angle and angle < ignore_angle_range_list_[i*2 + 1])
            return true;
    }

    if(size%2 == 1)
    {
        if(ignore_angle_range_list_[size-1] < angle)
            return true;
    }

    return false;
}
