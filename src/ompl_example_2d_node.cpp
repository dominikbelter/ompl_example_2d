#include <memory>
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <nav_msgs/msg/path.hpp> 

using std::placeholders::_1;
using namespace std::chrono_literals;

nav_msgs::msg::OccupancyGrid globalMap;

class MapSubscriber : public rclcpp::Node
{
  public:
    MapSubscriber()
    : Node("map_subscriber")
    {
      subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&MapSubscriber::map_callback, this, _1));
    }

  private:
    void map_callback(const nav_msgs::msg::OccupancyGrid & msg) const
    {
      globalMap = msg;
    }
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
};

class PathPublisher : public rclcpp::Node
{
  public:
    PathPublisher()
    : Node("path_publisher")
    {
      publisher_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&PathPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = nav_msgs::msg::Path();
      // message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: ");
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapSubscriber>());
  rclcpp::shutdown();
  return 0;
}