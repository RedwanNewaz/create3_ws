#include <cstdio>
#include "MapTransformer.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); 
  //logitec_cam_tag36h11:7
  auto logitec_to_tag = std::make_shared<airlab::MapTransformer>("logitec_cam", "tag36h11:7", 0.99);
  auto nexigo_to_tag = std::make_shared<airlab::MapTransformer>("nexigo_cam", "tag36h11:7", 0.99);
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(logitec_to_tag);
  executor.add_node(nexigo_to_tag);

  executor.spin();
  rclcpp::shutdown();
  
  return 0;
}
