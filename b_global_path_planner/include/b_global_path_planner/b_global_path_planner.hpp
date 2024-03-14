#ifndef GLOBAL_PATH_PLANNER_H
#define GLOBAL_PATH_PLANNER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <math.h>

class Astar: public rclcpp::Node
{

public:
    Astar();
    void process();
    int hz_;  //周波数
private:
    struct Node
    {
        int    x = 0;
        int    y = 0;
        int    parent_x = - 1;
        int    parent_y = -1;
        double f = 0.0;
    };

    struct Motion
    {
        int dx;
        int dy;
        double cost;
    };
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr mag);  //マップの読み込み
    void obs_expander();
    void obs_expand(const int index);
    void swap_node(const Node node, std::vector<Node>& list1,std::vector<Node>& list2);  //リスト間の移動
    void show_node_point(const Node node);  //ノードの表示
    void show_path(nav_msgs::msg::Path& current_path);  //パスの表示
    // void get_way_points(std::vector<std::vector<int>>& list);  //経由点の宣言
    void planning();  //経路計画
    void create_path(Node node);  //経路作成
    void update_list(const Node node);  //隣接ノードの更新
    void create_neighbors(const Node node, std::vector<Node>& nodes);  //隣接ノードの作成
    void get_motion(std::vector<Motion>& motion);  //動作の定義
    void show_exe_time();

    bool check_same_node(const Node n1, const Node n2);  //同じノードかどうかの確認
    bool check_obs(const Node node);  //壁か判断
    bool check_start(const Node node);  //スタートかどうか
    bool check_goal(const Node node);  //ゴールかどうか
    bool check_parent(const int index, const Node node);

    double make_heuristic(const Node node);  //ヒューリスティック関数の取得
    int sleep_time_;
    double margin_;
    int check_list(const Node target_node, std::vector<Node>& set);  //リストの中を検索
    int search_node_from_list(const Node node, std::vector<Node>& list);  //リストの中を検索
    Node set_way_point(const int phase);  //経由点の取得
    Node select_min_f();  //一番f値が小さいものを選択
    Node get_neighbor(const Node node, const Motion motion);  //隣接ノードの取得
    Motion motion(const int dx,const int dy,const int cost);  //もーしょん
    geometry_msgs::msg::PoseStamped node_to_pose(const Node node);  //ノードから座標に変換

    std::tuple<int, int> search_node(const Node node);  //どっちのリストに入っているのか検索

    double origin_x_;  //原点のx座標
    double origin_y_;  //原点のy座標
    std::vector<double> way_points_x_;  //経由点
    std::vector<double> way_points_y_;  //経由点


    Node start_node_;  //開始位置
    Node goal_node_;  //目標位置
    std::vector<Node> open_list_;  //opneリスト
    std::vector<Node> close_list_;  //closeリスト

    bool map_checker_ = false;  //正しくマップはとれたかな
    bool test_show_ = true;  //デバッグしますか

    //map製作用
    int height_;  //マップの幅だか高さだか
    int width_;  //マップの幅だか高さだか
    double resolution_;  //マップの解像度
    Node origin_;  //マップの原点
    std::vector<std::vector<int>> map_grid_;  //グリッドマップ

    rclcpp::Clock begin_;

    // rclcpp::NodeHandle nh_;
    // rclcpp::NodeHandle private_nh_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_node_point_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_current_path_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_new_map_;

    nav_msgs::msg::Path global_path_;
    nav_msgs::msg::OccupancyGrid map_;
    nav_msgs::msg::OccupancyGrid new_map_;
    geometry_msgs::msg::PointStamped current_node_;
};
#endif
