#include "b_obstacle_detector/b_obstacle_detector.hpp"

ObstacleDetector::ObstacleDetector()
: Node("b_obstacle_detector")
{
  
  timer_ = this->create_wall_timer(0.5s, std::bind(&ObstacleDetector::timer_callback, this));
  //auto hz_ = this->get_parameter("hz").as_int();
  ignore_dist_ = this->declare_parameter<double>("ignore_dist",0.01);
  laser_step_ = this->declare_parameter<int>("laser_step",3); 
  ignore_angle_range_list_ = this->declare_parameter<std::vector<double>>("ignore_angle_range_list",{(2.55* M_PI /16.0), (5.2*M_PI/16.0),(-3.4*M_PI/16.0),(-5.2*M_PI/16.0), (10.5*M_PI/16.0)});
  //printf("ignore_angle_range_list = %f\n",ignore_angle_range_list_[0]);

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",rclcpp::QoS(1).reliable(),std::bind(&ObstacleDetector::scan_callback,this,std::placeholders::_1));
  obstacle_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/obstacle_pose",rclcpp::QoS(1).reliable());

  // scan_ を初期化する
  scan_ = std::nullopt;

  obstacle_pose_array_.header.frame_id = "base_link";
}

void ObstacleDetector::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // 一定周期で行う処理を書く
    // printf("scan");
    scan_ = *msg;
    //printf("scan_[20] = %f\n", scan_.value().ranges[20]);
    flag_laser_scan_ = true;    
}

void ObstacleDetector::timer_callback()
{
    // 一定周期で行う処理を書く
    
    if(scan_.has_value()){
        //printf("timer\n");
        process();
    }
    
}


void ObstacleDetector::process()
{
    //ros::Rate loop_rate(hz_);
    //printf("process1\n");
    //nodeが続く間繰り返される
    /*
    while(rclcpp::ok())
    {
        //printf("process2\n"); 
        if(flag_laser_scan_)
        {
            //printf("process3\n");
            scan_obstacle();
        }
        //auto node_ = rclcpp::Node::make_shared("b_obstacle_detector");
        //auto node_ = std::make_shared<ObstacleDetector>();
        rclcpp::spin_some(node_);
        rclcpp::WallRate loop_rate(500ms);
        loop_rate.sleep();
    }
    */
    if(flag_laser_scan_)
    {
        //printf("process3\n");
        scan_obstacle();
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

        //printf("scan_obstacle\n");
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
        //double  = range * cos(angle);
        //printf("position\n");
        // printf("pose_array = %f\n", obstacle_pose_array_.poses[20].position.x);
    }
    // printf("pub\n");
    //printf("position_x = %f\n",);
    obstacle_pose_pub_->publish(obstacle_pose_array_);
}


//無視するlidarの範囲の決定
bool ObstacleDetector::is_ignore_scan(double angle)
{
    //printf("is_ignore\n");
    //angle = abs(angle);
    //const int size = ignore_angle_range_list_.size();

    if ((ignore_angle_range_list_[0] < angle) and (angle < ignore_angle_range_list_[1]))
    {
        // printf("ignore1\n");
        return true;
    }
    if ((ignore_angle_range_list_[2] > angle) and (angle > ignore_angle_range_list_[3]))
    {
        // printf("ignore1\n");
        return true;
    }

    if (abs(ignore_angle_range_list_[4]) < abs(angle))
    {
        // printf("ignore2\n");
        return true;
    }
        
    return false;
}
