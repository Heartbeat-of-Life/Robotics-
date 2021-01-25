#include <memory>

#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/detail/float32__struct.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("test_subscriber")
    {
	    std::cout <<"initializing test node"<<std::endl;
      subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "pendulum_wagon_force", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::Float32::SharedPtr msg) const
    {
	    std::cout<<"rec message in test node"<<std::endl;
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
