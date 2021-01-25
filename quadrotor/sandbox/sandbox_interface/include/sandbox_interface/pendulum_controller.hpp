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
	protected:
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
		T u{1};
		return u;
	}
};









#endif
