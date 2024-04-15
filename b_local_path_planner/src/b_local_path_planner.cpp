#include "b_local_path_planner/b_local_path_planner.hpp"

DWAPlanner::DWAPlanner() : Node("b_local_path_planner")
{
    // パラメータ宣言
    this->declare_parameter("hz", 50);
    this->declare_parameter("dt", 0.1);             // 変更の余地アリ
    this->declare_parameter("goal_tolerance", 0.5); // 変更の余地あり
    this->declare_parameter("max_vel", 0.1);
    this->declare_parameter("min_vel", 0.0);
    this->declare_parameter("max_yawrate", 0.1);
    this->declare_parameter("min_yawrate", -0.1);
    this->declare_parameter("max_accel", 1000.0);
    this->declare_parameter("max_dyawrate", 1000.0);
    this->declare_parameter("v_reso", 0.05);
    this->declare_parameter("y_reso", 0.02);
    this->declare_parameter("predict_time",3.0); // 変更の余地あり
    this->declare_parameter("heading_cost_gain", 0.7);
    this->declare_parameter("velocity_cost_gain", 0.6);
    this->declare_parameter("distance_cost_gain", 0.8);
    this->declare_parameter("robot_radius", 0.25); // いらない
    this->declare_parameter("radius_margin", 0.1); // いらない
    this->declare_parameter("search_range", 0.95);  // 変更の余地あり

    // パラメータの取得
    this->get_parameter("hz", hz_);
    this->get_parameter("dt", dt_);
    this->get_parameter("goal_tolerance", goal_tolerance_);
    this->get_parameter("max_vel", max_vel_);
    this->get_parameter("min_vel", min_vel_);
    this->get_parameter("max_yawrate", max_yawrate_);
    this->get_parameter("min_yawrate", min_yawrate_);
    this->get_parameter("max_accel", max_accel_);
    this->get_parameter("max_dyawrate", max_dyawrate_);
    this->get_parameter("v_reso", v_reso_);
    this->get_parameter("y_reso", y_reso_);
    this->get_parameter("predict_time", predict_time_);
    this->get_parameter("heading_cost_gain", heading_cost_gain_);
    this->get_parameter("velocity_cost_gain", velocity_cost_gain_);
    this->get_parameter("distance_cost_gain", distance_cost_gain_);
    this->get_parameter("robot_radius", robot_radius_);
    this->get_parameter("radius_margin", radius_margin_);
    this->get_parameter("search_range", search_range_);
    // this->declare_parameter<std::string>("robot_frame","base_link");
    
    // printf("hz =%d\n",hz_);
    // printf("dt =%f\n",dt_);
    // printf("predict_time =%f\n",predict_time_);
    // printf("goal_tolerance =%f\n",goal_tolerance_);
    //printf("max_vel =%f\n",max_vel_);
    // printf("min_vel =%f\n",min_vel_);
    // printf("max_yawrate =%f\n",max_yawrate_);
    // printf("min_yawrate =%f\n",min_yawrate_);
    // printf("max_accel =%f\n",max_accel_);
    // printf("max_dyawrate =%f\n",max_dyawrate_);
    // printf("v_reso =%f\n",v_reso_);
    // printf("y_reso =%f\n",y_reso_);
    // printf("heading_cost_gain =%f\n",heading_cost_gain_);
    // printf("velocity_cost_gain =%f\n",velocity_cost_gain_);
    // printf("distance_cost_gain =%f\n",distance_cost_gain_);
    // printf("serch_range =%f\n",search_range_);
    

    // Subscriber
    local_goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/local_goal", rclcpp::QoS(1).reliable(), std::bind(&DWAPlanner::local_goal_callback, this, std::placeholders::_1));
    obs_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/obstacle/pose", rclcpp::QoS(1).reliable(), std::bind(&DWAPlanner::obs_pose_callback, this, std::placeholders::_1));

    // Publisher
    cmd_speed_pub_ = this->create_publisher<roomba_500driver_meiji::msg::RoombaCtrl>("/roomba/control", rclcpp::QoS(1).reliable());
    predict_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/predict/path", rclcpp::QoS(1).reliable());
    optimal_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/optimal/path", rclcpp::QoS(1).reliable());

    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    // predict_path_.header.frame_id ="base_link";
    // optimal_path_.header.frame_id ="base_link";
}

void DWAPlanner::local_goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) // local goalを受け取る
{
    //printf("local_goal_callback\n");
    geometry_msgs::msg::TransformStamped transformStamped;
    try
    {
        // lookupTransform("変換のベースとなる座標系","変更したい対象の座標系",変更したい時間(過去データを扱う場合は注意が必要))
        // transformStamped = tfBuffer_->lookupTransform(this->get_parameter("robot_frame").as_string(),"map", tf2::TimePointZero); //座標系の変換
        //printf("1\n");
        transformStamped = tfBuffer_->lookupTransform("base_link", "map", tf2::TimePointZero); // 座標系の変換
        //printf("2\n");
        //  取得した変換情報を表示
        // RCLCPP_INFO(this->get_logger(), "Transform: [%f, %f, %f]", transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
        flag_local_goal_ = true;
         printf("transform\n");
    }
    catch (tf2::TransformException &ex) // エラー
    {
        RCLCPP_WARN(this->get_logger(), "goal_handle empty");
        flag_local_goal_ = false;
        return;
    }

    tf2::doTransform(*msg, local_goal_, transformStamped); // do座標変換
}

void DWAPlanner::obs_pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) // 障害物情報を受け取る
{
    obs_pose_ = *msg;
    flag_obs_pose_ = true;
    //printf("obs x=%f\n", obs_pose_.poses[2].position.x);
    //printf("obs y=%f\n", obs_pose_.poses[2].position.y);
    //printf("obs_pose_callback\n");
}

bool DWAPlanner::is_goal_reached() // 情報が適切にsubscribeされているか判定する
{
    if (not(flag_local_goal_ or flag_obs_pose_))
    {
        return false;
    }

    double dx = local_goal_.point.x;
    double dy = local_goal_.point.y;
    double dist = hypot(dx, dy);

    if (dist > goal_tolerance_)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void DWAPlanner::process() // 全体の処理(主要なループ)
{

    if (is_goal_reached())
    {
        const std::vector<double> input = calc_input(); // 計算(速度と回転角速度をinputで返す)
        // RCLCPP_WARN_STREAM ::重大度warnのメッセージをログに記録する
        // RCLCPP_WARN_STREAM(rclcpp::get_logger("b_local_path_planner"),"input[0]: " << input[0]);
        // RCLCPP_WARN_STREAM(rclcpp::get_logger("b_local_path_planner"),"input[1]: " << input[1]);
        roomba_ctl(input[0], input[1]); // roombaに速度,回転角速度指令
    }
    else
    {
        roomba_ctl(0.0, 0.0); // 止まる
    }
}

void DWAPlanner::roomba_ctl(double vel, double yawrate) // roomba制御
{

    roomba_ctl_msg_.mode = 11;
    roomba_ctl_msg_.cntl.linear.x = vel;
    roomba_ctl_msg_.cntl.angular.z = yawrate;

    cmd_speed_pub_->publish(roomba_ctl_msg_);
    //printf("roomba_ctl\n");
}

std::vector<double> DWAPlanner::calc_input() // 計算(速度と回転角速度をinputで返す)
{
    // ROS_WARN_STREAM("calc_input");
    std::vector<double> input{0.0, 0.0};
    std::vector<std::vector<State>> trajectory_list;
    double max_score = -1.0;
    int max_score_index = 0;
    calc_dynamic_window(); // roombaを制御するパラメータの範囲を決定

    int i = 0;
    for (double velocity = dw_.min_vel; velocity <= dw_.max_vel; velocity += v_reso_)
    {
        for (double yawrate = dw_.min_yawrate; yawrate <= dw_.max_yawrate; yawrate += y_reso_)
        {
            // RCLCPP_WARN_STREAM(rclcpp::get_logger("b_local_path_planner"),"max_yawrate: " << dw_.max_yawrate);
            // RCLCPP_WARN_STREAM(rclcpp::get_logger("b_local_path_planner"),"yawrate: " << yawrate);
            const std::vector<State> trajectory = calc_trajectory(velocity, yawrate); // 軌跡の計算
            double score = calc_eval(trajectory);                                     // 評価関数の最適化::評価関数を設定してスコアを計算する(最も適切な速度と回転角速度を探す)
            // ROS_WARN_STREAM("score: " << score);
            // ROS_WARN_STREAM("max_score: " << max_score);
            trajectory_list.push_back(trajectory);
            // ROS_INFO_STREAM("trajectory.back().x: " << trajectory.back().x);
            // ROS_INFO_STREAM("trajectory.back().y: " << trajectory.back().y);

            if (score > max_score) // スコア比較
            {
                max_score = score;
                max_score_index = i;
                input[0] = velocity;
                input[1] = yawrate;
                // ROS_WARN_STREAM("max_score: " << max_score);
            }
            // ROS_WARN_STREAM("i: " << i);
            i++;
        }
    }

    roomba_.vel = input[0];
    roomba_.yawrate = input[1];

    // rclcpp::Time now = rclcpp::Time::now();
    // rclcpp::Time now = ros_clock.now();
    rclcpp::Clock ros_clock(rcl_clock_type_t RCL_ROS_TIME);
    rclcpp::Time now = get_clock()->now();
    for (int i = 0; i < trajectory_list.size(); i++)
    {
        if (i == max_score_index)
        {
            // printf("optimal\n");
            visualize_trajectory(trajectory_list[i], optimal_path_pub_, now); // 最適ルート
        }
        else
        {
            // printf("predict\n");
            visualize_trajectory(trajectory_list[i], predict_path_pub_, now); // 想定されるルート
        }
    }

    return input;
}

void DWAPlanner::calc_dynamic_window() // roombaを制御するパラメータの範囲を決定
{
    // RCLCPP_WARN_STREAM(rclcpp::get_logger("b_local_path_planner"),"calc_dynamic_window");
    // 制御可能範囲::ロボットが取りうる制御入力の最大値と最小値の範囲を意味する
    double Vs[] = {min_vel_, max_vel_, min_yawrate_, max_yawrate_}; // roombaの性能(速度,回転角速度)
    // 動的制御可能範囲::現在の速度とスペック上の最大加速度から計算した次の時刻に取りうる最大最小の制御入力
    double Vd[] = {roomba_.vel - max_accel_ * dt_,
                   roomba_.vel + max_accel_ * dt_,
                   roomba_.yawrate - max_dyawrate_ * dt_,
                   roomba_.yawrate + max_dyawrate_ * dt_}; // roombaの

    dw_.min_vel = std::max(Vs[0], Vd[0]);
    dw_.max_vel = std::min(Vs[1], Vd[1]);
    dw_.min_yawrate = std::max(Vs[2], Vd[2]);
    dw_.max_yawrate = std::min(Vs[3], Vd[3]);
    // ROS_WARN_STREAM("dw_.min_vel: " << dw_.min_vel);
    // ROS_WARN_STREAM("dw_.max_vel: " << dw_.max_vel);
    // ROS_WARN_STREAM("dw_.min_yawrate: " << dw_.min_yawrate);
    // ROS_WARN_STREAM("dw_.max_yawrate: " << dw_.max_yawrate);
}

std::vector<State> DWAPlanner::calc_trajectory(double vel, double yawrate) // 軌跡の計算
{
    std::vector<State> trajectory;
    State state = {0.0, 0.0, 0.0, 0.0, 0.0};

    for (double t = 0.0; t <= predict_time_; t += dt_)
    {
        move_robot_image(state, vel, yawrate); // 微小時間後のroomba状態
        trajectory.push_back(state);
    }

    // ROS_WARN_STREAM("trajectory.back().x: " << trajectory.back().x);
    // ROS_WARN_STREAM("trajectory.back().y: " << trajectory.back().y);
    return trajectory;
}

void DWAPlanner::move_robot_image(State &state, double vel, double yawrate) // 微小時間後のroomba状態
{
    // ROS_WARN_STREAM("vel: " << vel);
    // ROS_WARN_STREAM("yawrate: " << yawrate);
    // ROS_WARN_STREAM("dt_: " << dt_);
    state.x += vel * cos(state.yaw) * dt_;
    state.y += vel * sin(state.yaw) * dt_;
    state.yaw += yawrate * dt_;
    state.yaw = nomalize_angle(state.yaw);
    state.vel = vel;
    state.yawrate = yawrate;
}

double DWAPlanner::nomalize_angle(double angle) // angleの標準化
{
    if (angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    else if (angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }

    return angle;
}

double DWAPlanner::calc_eval(const std::vector<State> &trajectory) // 評価関数の最適化::評価関数を設定してスコアを計算する(最も適切な速度と回転角速度を探す)

{
    double heading = calc_heading_eval(trajectory);   // 方向評価
    double velocity = calc_velocity_eval(trajectory); // 速度評価
    double distance = calc_distance_eval(trajectory); // 障害物までの距離評価

    // ROS_INFO_STREAM("heading: " << heading);
    // ROS_INFO_STREAM("velocity: " << velocity);
    // ROS_INFO_STREAM("distance: " << distance);

    // RCLCPP_INFO(rclcpp::get_logger("b_local_path_planner"),"heading: %f" , heading_cost_gain_*heading);
    // RCLCPP_INFO(rclcpp::get_logger("b_local_path_planner"),"velocity: %f" , velocity_cost_gain_*velocity);
    // RCLCPP_INFO(rclcpp::get_logger("b_local_path_planner"),"distance: %f" , distance_cost_gain_*distance);

    return heading_cost_gain_ * heading + velocity_cost_gain_ * velocity + distance_cost_gain_ * distance;
}

double DWAPlanner::calc_heading_eval(const std::vector<State> &trajectory) // 方向評価 ゴールの方を向いているほど高評価
{
    //printf("heading\n");
    double dx = local_goal_.point.x - trajectory.back().x;
    double dy = local_goal_.point.y - trajectory.back().y;
    double goal_yaw = atan2(dy, dx);
    double yaw = trajectory.back().yaw;
    double heading = 0.0;
    if (goal_yaw > yaw)
    {
        heading = goal_yaw - yaw;
    }
    else
    {
        heading = yaw - goal_yaw;
    }

    return (M_PI - abs(nomalize_angle(heading - M_PI))) / M_PI;
}

double DWAPlanner::calc_velocity_eval(const std::vector<State> &trajectory) // 速度評価 速度が速い制御入力が高評価
{
    //printf("velocity\n");
    if (0.0 < trajectory.back().vel and trajectory.back().vel < max_vel_)
    {
        // printf("velocity = %f\n",trajectory.back().vel);
        return trajectory.back().vel / max_vel_;
    }
    else
    {
        return 0.0;
    }
}

double DWAPlanner::calc_distance_eval(const std::vector<State> &trajectory) // 障害物までの距離評価 障害物から遠い制御入力が高評価
{
    //printf("distance\n");
    double min_dist = search_range_;

    for (const auto &state : trajectory)
    {
        //printf("state\n");
        for (const auto &obs : obs_pose_.poses)
        {
            //printf("obs\n");
            double dx = state.x - obs.position.x;
            double dy = state.y - obs.position.y;
            double dist = hypot(dx, dy);
            //printf("dist = %f\n", dist);

            if (dist < min_dist)
            {
                min_dist = dist;
            }
        }
    }
    return min_dist / search_range_;
}

void DWAPlanner::visualize_trajectory(const std::vector<State> &trajectory, const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &local_path_pub_, rclcpp::Time now) // 軌跡の表示
{

    nav_msgs::msg::Path local_path;
    local_path.header.stamp = now;
    local_path.header.frame_id = "base_link";

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = "base_link";

    for (const auto &state : trajectory)
    {
        pose.pose.position.x = state.x;
        pose.pose.position.y = state.y;
        // std::cout << "x: " << state.x << std::endl;
        // std::cout << "y: " << state.y << std::endl;
        local_path.poses.push_back(pose);
    }
    local_path_pub_->publish(local_path);
}

/*
void DWAPlanner::debager()
{
    std::shared_ptr<rclcpp::Node> node_ptr = rclcpp::Node::make_shared("ros_node_name");
    RCLCPP_ERROR(node_ptr_->get_logger(), "Error Message");

}
*/