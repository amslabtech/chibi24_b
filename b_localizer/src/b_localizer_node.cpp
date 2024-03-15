#include "b_localizer/b_localizer.hpp"

// ===== メイン関数 =====
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv); // ノードの初期化
    EMCL emcl;
    emcl.process();

    return 0;
}