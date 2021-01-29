#include "messages/srv/detail/pendulum_sensor_info__struct.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sandbox_interface/sandbox_interface.hpp"
#include <iterator>

using namespace std::chrono_literals;
namespace sbx
{
	PendulumController::PendulumController():Node("pendulum_controller")
	{
		timer = this->create_wall_timer(100ms, std::bind(&PendulumController::timer_callback, this));	
		this->client =this->create_client<messages::srv::PendulumSensorInfo>("pendulum_sensor");
		publisher = this->create_publisher<std_msgs::msg::Float64>("control_force",10);
		PendulumParams<double> params;
		this->ptr_controller = std::make_unique<PendulumControllerOPLin<double>>(params);
	}

	void PendulumController:: compute_controls()
	{
	}

	void PendulumController:: timer_callback()
	{
		    auto request = std::make_shared<messages::srv::PendulumSensorInfo::Request>();
		    request->t=clock.now().seconds();
		    using SharedResponse=typename messages::srv::PendulumSensorInfo::Response::SharedPtr;
		    using SharedFuture = std::shared_future<SharedResponse>;
		    auto callback = [this](SharedFuture ptr){
			    PendulumControllerBase<double>::state_type state;
			    state(0)=ptr.get()->x;
			    state(1)=ptr.get()->dx;
			    state(2)=ptr.get()->phi;
			    state(3)=ptr.get()->dphi;
			    this->ptr_controller->set_state(state);
			    auto message = std_msgs::msg::Float64();
			    message.data=ptr_controller->compute_controls();
			    this->publisher->publish(message);
		    };
		    auto result = this->client->async_send_request(request,callback);
	 }


}

  int main(int argc, char * argv[])
  {
	  rclcpp::init(argc,argv);
	  rclcpp::spin(std::make_shared<sbx::PendulumController>());
	  rclcpp::shutdown();
    	  return 0;
  }

