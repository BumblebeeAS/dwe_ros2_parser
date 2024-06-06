#include "dwe_ros2_parser/dwe_ros2_parser.hh"

// Start DWE_Ros2_Parser 
int main(int args, char* argv[]) {
    rclcpp::init(args, argv);
    rclcpp::spin(make_shared<DWE_Ros2_Parser>());
    rclcpp::shutdown();
    return 1;
}