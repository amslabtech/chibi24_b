#include "b_global_path_planner/b_global_path_planner.hpp"

Astar::Astar(): rclcpp::Node("b_global_path_planner") {
    hz_ = this->declare_parameter<int>("hz", 10);
    test_show_ = this->declare_parameter<bool>("test_show", true);
    way_points_x_ = this->declare_parameter<std::vector<double>>("way_points_x", {0.0, 6.24, 11.92, 12.81, 13.23, 13.60, 10.75, 1.69, -7.27, -18.32, -20.14, -20.38, -20.50, -15.11, -6.95, 0.0});
    way_points_y_ = this->declare_parameter<std::vector<double>>("way_points_y", {0.0, -0.38, -0.57, 2.36, 6.46, 12.54, 13.80, 14.11, 14.49, 15.14, 11.18, 7.24, 1.85, 0.68, 0.34, 0.0});
    margin_ = this->declare_parameter<double>("margin", 0.25);
    sleep_time_ = this->declare_parameter<int>("sleep_time", 500000000);

    sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).reliable(), std::bind(&Astar::map_callback, this, std::placeholders::_1));

    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", rclcpp::QoS(1));
    pub_new_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map/new_map", rclcpp::QoS(1));

    if (test_show_) {
        pub_current_path_ = this->create_publisher<nav_msgs::msg::Path>("/current_path", rclcpp::QoS(1));
        pub_node_point_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/current_node", rclcpp::QoS(1));
    }

    global_path_.header.frame_id = "map";
    current_node_.header.frame_id = "map";

    global_path_.poses.reserve(2000);
}


void Astar::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_ = *msg;
    origin_x_ = map_.info.origin.position.x;
    origin_y_ = map_.info.origin.position.y;
    height_ = map_.info.height;
    width_ = map_.info.width;
    resolution_ = map_.info.resolution;
    map_checker_ = true;
}


void Astar::obs_expander() {
    sleep(2);
    new_map_ = map_;
    int grid_size = new_map_.data.size();
    for (int i = 0; i < grid_size; i++) {
        if (map_.data[i] == 100) {
            obs_expand(i);
        }
    }
    pub_new_map_->publish(new_map_);
}

void Astar::obs_expand(const int index) {
    int index_x = index % width_;
    int index_y = index / width_;
    int margin_length = round(margin_ / resolution_);
    for (int i = -margin_length; i < margin_length; i++) {
        for (int j = -margin_length; j < margin_length; j++) {
            int grid_index = (index_x + j) + ((index_y + i) * width_);
            new_map_.data[grid_index] = 100;
        }
    }
}

Astar::Node Astar::set_way_point(const int phase) {
    Node way_point;
    way_point.x = round((way_points_x_[phase] - origin_x_) / resolution_);
    way_point.y = round((way_points_y_[phase] - origin_y_) / resolution_);
    return way_point;
}

double Astar::make_heuristic(const Node node) {
    const double dx = (node.x - goal_node_.x);
    const double dy = (node.y - goal_node_.y);
    const double distance = hypot(dx, dy);
    return distance;
}

bool Astar::check_same_node(const Node n1, const Node n2) {
    return n1.x == n2.x and n1.y == n2.y;
}

int Astar::check_list(const Node target_node, std::vector<Node>& set) {
    for (int i = 0; i < set.size(); i++) {
        if (check_same_node(target_node, set[i])) {
            return i;
        }
    }
    return -1;
}

bool Astar::check_obs(const Node node) {
    const int grid_index = node.x + (node.y * width_);
    return new_map_.data[grid_index] == 100;
}

void Astar::swap_node(const Node node, std::vector<Node>& list1, std::vector<Node>& list2) {
    const int list1_node_index = check_list(node, list1);
    if (list1_node_index == -1) {
        exit(0);
    }

    list1.erase(list1.begin() + list1_node_index);
    list2.push_back(node);
}

void Astar::show_node_point(const Node node) {
    if (test_show_) {
        current_node_.point.x = node.x * resolution_ + origin_x_;
        current_node_.point.y = node.y * resolution_ + origin_y_;
        pub_node_point_->publish(current_node_);
        // node.cppで時間制御
        // rclcpp::Duration::from_nanoseconds(sleep_time_).sleep();
    }
}

void Astar::show_path(nav_msgs::msg::Path& current_path) {
    if (test_show_) {
        current_path.header.frame_id = "map";
        pub_current_path_->publish(current_path);
        // node.cpp で時間制御
        // rclcpp::Duration::from_nanoseconds(sleep_time_).sleep();
    }
}

Astar::Node Astar::select_min_f() {
    Node min_node = open_list_[0];
    double min_f = open_list_[0].f;

    for(const auto& open_node : open_list_) {
        if(open_node.f < min_f) {
            min_f = open_node.f;
            min_node = open_node;
        }
    }
    return min_node;
}

bool Astar::check_goal(const Node node) {
    return check_same_node(node,goal_node_);
}

bool Astar::check_start(const Node node) {
    return check_same_node(node,start_node_);
}

void Astar::update_list(const Node node) {
    std::vector<Node> neighbors;
    create_neighbors(node, neighbors);

    for (const auto& neighbor : neighbors) {
        if (check_obs(neighbor)) {
            continue;
        }

        int flag;
        int node_index;
        std::tie(flag, node_index) = search_node(neighbor);


        if(flag == -1)
        {
            open_list_.push_back(neighbor);
        }
        else if(flag == 1)
        {
            if(neighbor.f < open_list_[node_index].f)
            {
                open_list_[node_index].f = neighbor.f;
                open_list_[node_index].parent_x = neighbor.parent_x;
                open_list_[node_index].parent_y = neighbor.parent_y;
            }

        }
        else if(flag == 2)
        {
            if(neighbor.f < close_list_[node_index].f)
            {
                close_list_.erase(close_list_.begin() + node_index);
                open_list_.push_back(neighbor);
            }
        }
    }
}

void Astar::create_neighbors(const Node node, std::vector<Node>&  neighbors)  //隣接ノードの作成
{
    std::vector<Motion> motion_list;
    get_motion(motion_list);
    const int motion_num = motion_list.size();

    for(int i=0;i<motion_num;i++)
    {
        Node neighbor = get_neighbor(node, motion_list[i]);
        neighbors.push_back(neighbor);
    }
}

void Astar::get_motion(std::vector<Motion>& list)  //動きの取得
{
    list.push_back(motion(1,0,1));
    list.push_back(motion(0,1,1));
    list.push_back(motion(-1,0,1));
    list.push_back(motion(0,-1,1));

    list.push_back(motion(1,1,sqrt(2)));
    list.push_back(motion(1,-1,sqrt(2)));
    list.push_back(motion(-1,1,sqrt(2)));
    list.push_back(motion(-1,-1,sqrt(2)));
}

Astar::Motion Astar::motion(const int dx,const int dy,const int cost)  //もーしょん
{
    Motion motion;
    motion.dx = dx;
    motion.dy = dy;
    motion.cost = cost;

    return motion;
}

Astar::Node Astar::get_neighbor(const Node node, const Motion motion)  //隣接ノードの取得
{
    Node neighbor;

    neighbor.x = node.x + motion.dx;
    neighbor.y = node.y + motion.dy;

    neighbor.f = (node.f - make_heuristic(node)) + make_heuristic(neighbor) + motion.cost;

    neighbor.parent_x = node.x;
    neighbor.parent_y = node.y;

    return neighbor;
}

std::tuple<int, int> Astar::search_node(const Node node)  //どこのリストに入っているかの検索
{
    const int open_list_index = search_node_from_list(node,open_list_);
    if(open_list_index != -1)
    {
        return std::make_tuple(1,open_list_index);
    }

    const int close_list_index = search_node_from_list(node,close_list_);
    if(close_list_index != -1)
    {
        return std::make_tuple(2,close_list_index);
    }
    return std::make_tuple(-1,-1);
}

void Astar::create_path(Node node)  //パスの作成
{
    nav_msgs::msg::Path partial_path;
    partial_path.poses.push_back(node_to_pose(node));
    int count = 0;

    while(not check_start(node))
    {
        for(int i=0;i<close_list_.size();i++)
        {
            if(check_parent(i,node))
            {
                node = close_list_[i];
                show_node_point(node);
                partial_path.poses.push_back(node_to_pose(node));
                break;
            }
            if(i == close_list_.size()-1)
            {
                // ROS_INFO("sippai.......");
                exit(0);
            }
        }
    }
    reverse(partial_path.poses.begin(),partial_path.poses.end());
    show_path(partial_path);
    global_path_.poses.insert(global_path_.poses.end(),partial_path.poses.begin(),partial_path.poses.end());
}

bool Astar::check_parent(const int index, const Node node)  //親ノードの確認
{
    bool check_x = close_list_[index].x == node.parent_x;
    bool check_y = close_list_[index].y == node.parent_y;

    return check_x and check_y;
}

geometry_msgs::msg::PoseStamped Astar::node_to_pose(const Node node)  //ノードから座標の検索
{
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = node.x * resolution_ + origin_x_;
    pose_stamped.pose.position.y = node.y * resolution_ + origin_y_;
    return pose_stamped;
}

void Astar::show_exe_time()
{
    //  ROS_INFO_STREAM("Duration = " << std::fixed << std::setprecision(2) << rclcpp::Time::now().toSec() - begin_.toSec() << "s");
}

int Astar::search_node_from_list(const Node node, std::vector<Node>& list)  //リストの中を検索
{
    for(int i=0;i<list.size();i++)
        if(check_same_node(node, list[i]))
        {
            return i;
        }
    return -1;
}

void Astar::planning()  //経路計画
{
    begin_ = rclcpp::Clock::now();
    const int total_phase = way_points_x_.size();
    for(int phase=0;phase<total_phase-1;phase++)
    {
        open_list_.clear();
        close_list_.clear();

        start_node_ = set_way_point(phase);
        goal_node_ = set_way_point(phase + 1);
        start_node_.f = make_heuristic(start_node_);
        open_list_.push_back(start_node_);

        while(rclcpp::ok())
        {
            Node min_node = select_min_f();
            show_node_point(min_node);

            if(check_goal(min_node))
            {
                create_path(min_node);
                break;
            }
            else
            {
                swap_node(min_node,open_list_,close_list_);
                update_list(min_node);
            }
        }
    }
    pub_path_->publish(global_path_);
    show_exe_time();
    // ROS_INFO("COMPLITE ASTAR PROGLAM");
    exit(0);
}

void Astar::process()  //メイン関数で実行する関数
{

    if (!map_checker_)
    {
        // ROS_INFO("NOW LOADING...");
    }
    else
    {
        obs_expander();
        planning();
    }

}
