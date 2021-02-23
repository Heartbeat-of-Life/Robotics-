//#include "messages/srv/detail/pendulum_sensor_info__struct.hpp"
#include "messages/msg/detail/control__struct.hpp"
#include "messages/msg/detail/predictive_control__struct.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sandbox_interface/sandbox_interface.hpp"
#include <iterator>
using namespace std::chrono_literals;
namespace sbx
{
	PendulumController::PendulumController():Node("pendulum_controller")
	{
//		std::cout <<"creating controller "<<std::endl;
		timer = this->create_wall_timer(10ms, std::bind(&PendulumController::timer_callback, this));	
		this->client =this->create_client<messages::srv::PendulumSensorInfo>("pendulum_sensor");
		this->publisher = this->create_publisher<cmtype>("control_force",10);
		PendulumParams<double> params;
		params.g=9.81;
		params.l=1;
		params.mh=1;
		params.ms=0;//1e-9;
		params.mw=1;
		params.xmax=1.5;
		params.xmax=-1.5;
		params.J=0;//0.000001;
		this->ptr_controller = std::make_unique<PendulumControllerOPLin<double>>(params);
		this->ptr_controller = std::make_unique<PendulumControllerMPC<double>>(params);
		this->ptr_controller->init();
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
		    const double tnow = this->clock.now().seconds();	
		    const auto vec=ptr_controller->compute_controls();
		    messages::msg::PredictiveControl message = messages::msg::PredictiveControl();
		    for(const auto &it : vec)
		    {
			    messages::msg::Control control;
			    control.t = it.t + tnow;
//			    std::cout<<"filling timestamps: it.t="<<it.t<<std::endl;
			    control.u = it.u;
			    message.uarray.emplace_back(control);
		    }
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
