#include "b_global_path_planner/b_global_path_planner.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto astar_node = std::make_shared<Astar>();
    astar_node->process();
    rclcpp::shutdown();

    return 0;
}