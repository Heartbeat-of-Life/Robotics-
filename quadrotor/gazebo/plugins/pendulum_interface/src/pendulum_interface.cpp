#ifndef _PENDULUM_PLUGIN_HH_
#define _PENDULUM_PLUGIN_HH_

#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/JointState.hh"
#include "ignition/math/Angle.hh"
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float64.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math.hh>
#include<std_msgs/msg/float32.hpp>
#include<messages/srv/pendulum_sensor_info.hpp>
#include<thread>

using namespace std::chrono_literals;

namespace gazebo
{
  class PendulumSensor : public ModelPlugin
  {
	public: PendulumSensor()  {
		
		
		auto service_callback = [this](const std::shared_ptr<messages::srv::PendulumSensorInfo::Request> req,
						     std::shared_ptr<messages::srv::PendulumSensorInfo::Response> res)->auto
		{ 
			res->t = clock.now().seconds();   
			const  ignition::math::v6::Pose3<double>pose = this->wagon->WorldPose();
			res->x=this->wagon->WorldPose().Pos().Y();
			res->dx=this->wagon->WorldLinearVel().Y();
			res->phi = this->model->GetLink("pendulum_stick")->WorldPose().Rot().Roll();
			res->dphi= this->pendulum_joint->GetVelocity(0);
			return;

		};
		this->node_ptr = rclcpp::Node::make_shared("PendulumSensor");
		this->service=node_ptr->create_service<messages::srv::PendulumSensorInfo>("pendulum_sensor",service_callback);
		auto cb = [this](const std_msgs::msg::Float64::SharedPtr msg){
			current_force = msg.get()->data;
		};
		this->sub = this->node_ptr->create_subscription<std_msgs::msg::Float64>("control_force",10,cb);
	}
	public: void onUpdate()
	{
		this->wagon_rail_joint->SetForce(0,current_force);
	}

	public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{
		this->model=_model;
		this->wagon = model->GetLink("pendulum_wagon");
		this->pendulum_joint = model->GetJoint("pendulum_to_wagon");
//		this->wagon_rail_joint = model->GetJoint("table_plate_to_wagon");
		this->update_connection=event::Events::ConnectWorldUpdateBegin(std::bind(&PendulumSensor::onUpdate,this));
		if(!rclcpp::ok())
		{
			rclcpp::init(0,nullptr);
		}
		this->queue_thread=std::thread(std::bind(&PendulumSensor::QueueThread,this));
	}

	private: void OnMsg(ConstVector3dPtr &_msg)
	{
	    std::cout <<"called pougin callback"<<std::endl;
	}

	private: void QueueThread()
	{
		rclcpp::spin(node_ptr);
		rclcpp::shutdown();
	}
	private: rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub;
	private: physics::ModelPtr model;
	private: physics::LinkPtr wagon;
	private: physics::JointPtr pendulum_joint;
	private: physics::JointPtr wagon_rail_joint;
	private: rclcpp::Node::SharedPtr node_ptr;
	private: std::thread queue_thread;
	private: rclcpp::Service<messages::srv::PendulumSensorInfo>::SharedPtr service;
	private: rclcpp::Clock clock;
	private: event::ConnectionPtr update_connection;
	private: double current_force;

};
	GZ_REGISTER_MODEL_PLUGIN(PendulumSensor)
}
#endif
