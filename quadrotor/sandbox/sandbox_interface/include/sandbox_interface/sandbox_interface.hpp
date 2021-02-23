#ifndef PENDULUM_CONTROLLER
#define PENDULUM_CONTROLLER


#include "messages/srv/detail/pendulum_sensor_info__struct.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include<messages/srv/pendulum_sensor_info.hpp>
#include "eigen3/Eigen/Dense"
#include"sandbox_interface/pendulum_controller.hpp"
#include"std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;
using cmtype = messages::msg::PredictiveControl;
namespace sbx
{
	class PendulumController: public rclcpp::Node
	{
		public: PendulumController(); 
		private: void compute_controls();
		private: void timer_callback();
		private:
		rclcpp::TimerBase::SharedPtr timer;
		rclcpp::Client<messages::srv::PendulumSensorInfo>::SharedPtr client;
		rclcpp::Clock clock;
		rclcpp::Publisher<cmtype>::SharedPtr publisher;
		std::unique_ptr<PendulumControllerBase<double>> ptr_controller;
	};
}

#endif
