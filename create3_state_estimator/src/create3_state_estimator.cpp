#include "JointStateEstimator.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto stateEstimator = std::make_shared<model::JointStateEstimator> ("stateEstimator");
     rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(stateEstimator);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}