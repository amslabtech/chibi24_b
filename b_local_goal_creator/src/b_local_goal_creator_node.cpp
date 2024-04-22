#include "b_local_goal_creator/b_local_goal_creator.hpp"

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv); // ノードの初期化
    auto node = std::make_shared<LocalGoalCreator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}