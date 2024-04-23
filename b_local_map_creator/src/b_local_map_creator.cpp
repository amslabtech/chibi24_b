#include "b_local_map_creator/b_local_map_creator.hpp"

LocalMapCreator::LocalMapCreator() : Node("b_local_map_creator")
{
    timer_ = this->create_wall_timer(0.5s, std::bind(&LocalMapCreator::timer_callback, this));
    hz_ = this->declare_parameter<int>("hz",10);
    //printf("hz = %ld\n",hz_);
    local_map_size_ = this->declare_parameter<double>("map_size",5);
    //printf("map_size = %f\n",local_map_size_);
    local_map_resolution_ = this->declare_parameter<double>("map_resolution",0.025);
    //printf("map_resolution = %f\n",local_map_resolution_);

    sub_obs_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/obstacle_pose",rclcpp::QoS(1).reliable(),std::bind(&LocalMapCreator::obs_poses_callback,this,std::placeholders::_1));
    pub_local_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/local/map",rclcpp::QoS(1).reliable());

    local_map_.info.resolution = local_map_resolution_;
    local_map_.info.width = int(round(local_map_size_ / local_map_resolution_));
    local_map_.info.height = int(round(local_map_size_ / local_map_resolution_));
    local_map_.info.origin.position.x = -local_map_size_ / 2.0;
    local_map_.info.origin.position.y = -local_map_size_ / 2.0;

    local_map_.data.reserve(local_map_.info.width * local_map_.info.height);
    local_map_.header.frame_id = "base_link";

}

void LocalMapCreator::obs_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    obs_poses_ = *msg;
    //printf("pose_array1 = %f\n",obs_poses_.poses[20].position.x);
    is_get_obs_poses_ = true;
}

void LocalMapCreator::timer_callback()
{
    // 一定周期で行う処理を書く
    process();
}

void LocalMapCreator::process()
{
    //ros::Rate loop_rate(hz_);
/*
    while(rclcpp::ok())
    {
        if(is_get_obs_poses_)
        {
            init_local_map();
            update_local_map();
            pub_local_map_->publish(local_map_);
        }
        auto node_ = rclcpp::Node::make_shared("b_local_map_creator_node");
        rclcpp::spin_some(node_);
        rclcpp::WallRate loop_rate(500ms);
        loop_rate.sleep();
        //loop_rate.sleep();
    }

*/
    if(is_get_obs_poses_)
    {
        //printf("map\n");
        //init_local_map();]
        //printf("local_map_info?_height = %d\n",local_map_.info.height);
        update_local_map();
        
    }
    
}
/*
void LocalMapCreator::init_local_map()
{
    local_map_.data.clear();
    local_map_.data.assign(local_map_.info.width * local_map_.info.height, -1);

}
*/

void LocalMapCreator::update_local_map()
{
    init_map(); //マップの初期化

    for(const auto & obs_pose : obs_poses_.poses)
    {
        const double x = obs_pose.position.x;
        const double y = obs_pose.position.y;
        // printf("x = %f\n",x);
        // printf("y = %f\n",y);
        const double dist = hypot(x, y);
        const double angle = atan2(y, x);

            for(double dist_from_robot=0.0; (dist_from_robot<dist && is_in_local_map(dist_from_robot, angle)); dist_from_robot+=local_map_resolution_)
            {
                const int grid_index = get_grid_index(dist_from_robot,angle);
                local_map_.data[grid_index] = 0;
            }
            if(is_in_local_map(dist, angle))
            {
                //printf("true\n");
                const int grid_index = xy_to_index(x,y);
                local_map_.data[grid_index] = 100;
                //printf("data = %d\n",local_map_.data[4]);
            }
    }
    pub_local_map_->publish(local_map_);
}

// マップの初期化(すべて「未知」にする)
void LocalMapCreator::init_map()
{
    local_map_.data.clear();

    const int size = local_map_.info.width * local_map_.info.height;
    for(int i=0; i<size; i++)
    {
        local_map_.data.push_back(-1); //「未知」にする
    }
}


bool LocalMapCreator::is_in_local_map(const double dist, const double angle)
{
    const double x = dist * cos(angle);
    const double y = dist * sin(angle);
    const int index_x = int(round((x - local_map_.info.origin.position.x) / local_map_.info.resolution));
    const int index_y = int(round((y - local_map_.info.origin.position.y) / local_map_.info.resolution));

    if((index_x < local_map_.info.width) && (index_x >= 0) && (index_y < local_map_.info.height) && (index_y >= 0))
    {
        return true;
    }
    else
    {
        return false;
    }
}

int LocalMapCreator::get_grid_index(const double dist, const double angle)
{
    const double x = dist * cos(angle);
    const double y = dist * sin(angle);
    int index = xy_to_index(x, y);
    return index;
}

int LocalMapCreator::xy_to_index(const double x, const double y)
{
    const int index_x = int(round((x - local_map_.info.origin.position.x) / local_map_.info.resolution));
    const int index_y = int(round((y - local_map_.info.origin.position.y) / local_map_.info.resolution));

    int index = index_y * local_map_.info.width + index_x;
    return index;
}