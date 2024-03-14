#include "b_obstacle_detector/b_obstacle_detector.hpp"

ObstacleDetector::ObstacleDetector(const std::string& b_obstacle_detector_topic,const std::string& b_obstacle_detector,const rclcpp::NodeOptions & node_options)
: Node("b_obstacle_detector",node_options)
{
  // timer
  // timer_ = this->create_wall_timer(<周期間隔>, std::bind(&<class名>::<callback関数名, this));
  auto hz_ = this->get_parameter("hz").as_int();
  auto ignore_dist_ = this->get_parameter("ignore_dist").as_double();
  auto laser_step_ = this->get_parameter("laser_step").as_int(); 
  auto ignore_angle_range_list_ = this->get_parameter("ignore_angle_range_list").as_double();



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
        //rclcpp::spin_some();
        //loop_rate.sleep();
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

        obstacle_pose_array_.poses[i].position.x = range * cos(angle);
        obstacle_pose_array_.poses[i].position.y = range * sin(angle);
        //obstacle_pose_array_.poses.push_back(obs_pose);

    }
    //obstacle_pose_pub_.publish(obstacle_pose_array_);
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
