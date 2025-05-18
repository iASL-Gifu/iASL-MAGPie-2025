#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class MapRepeaterNode : public rclcpp::Node {
public:
  MapRepeaterNode()
  : Node("map_repeater_node")
  {
    this->declare_parameter("repeat_rate", 1.0);
    double repeat_rate = this->get_parameter("repeat_rate").as_double();

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&MapRepeaterNode::map_callback, this, std::placeholders::_1)
    );

    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / repeat_rate),
      std::bind(&MapRepeaterNode::timer_callback, this)
    );
  }

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_map_ = msg;
  }

  void timer_callback() {
    if (latest_map_) {
      map_pub_->publish(*latest_map_);
    }
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapRepeaterNode>());
  rclcpp::shutdown();
  return 0;
}
