#ifndef PENDULUM_OP_STAB_CONTROLLER
#define PENDULUM_OP_STAB_CONTROLLER
#include "eigen3/Eigen/Dense"
#include "sandbox_interface/pendulum_params.hpp"
#include <cmath>
#include<iostream>
#include <string>
template<typename T>
class PendulumControllerBase 
{
	public:
	using state_type = Eigen::Matrix<T,4,1>;
	PendulumControllerBase() = delete;
	PendulumControllerBase(const PendulumParams<T> pendulum_params):pendulum_params(pendulum_params)
	{
	}
	virtual T compute_controls() const = 0;
	void set_state(const state_type& state_)
	{
		state=state_;
	}

	void set_wagon_mass(const T&mass_)
	{
		pendulum_params.mw = mass_;
	}
	void set_pendulum_head_mass(const T&mass_)
	{
		pendulum_params.mh = mass_;
	}
	void set_limits(const T&lower,const T&upper)
	{
		pendulum_params.xmin=lower;
		pendulum_params.xmax=upper;
	}
	protected:


	void init()
	{
		//compute inertia for instance
	}
	PendulumParams<T> pendulum_params;
	state_type state;
};


template<typename T>
class PendulumControllerOPLin : public PendulumControllerBase<T>
{
	public:
	PendulumControllerOPLin(const PendulumParams<T> pendulum_params):PendulumControllerBase<T>(pendulum_params)
	{
	}
	T compute_controls() const override
	{
		const PendulumParams<T>& p=this->pendulum_params;
//		std::cout <<"mw="<<p.mw<<" p.mh="<<p.mh<<" xmin="<<p.xmin<<" xmax="<<p.xmax<<std::endl;
//		std::cout<<"x="<<this->state(0)<<" dx="<<this->state(1)<<" phi="<<this->state(2)<<" dphi="<<this->state(3)<<std::endl;
//		T u = std::sin(10*this->state(0))*0;
		T u{1};
		return u;
	}
};









#endif
