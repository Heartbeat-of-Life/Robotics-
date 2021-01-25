#include "sandbox_interface/pendulum_controller.hpp"



template<typename T>
PendulumControllerBase<T>::PendulumControllerBase(PendulumParams<T> params): params(params)
{
}





template<typename T>
PendulumControllerOPLin<T>::PendulumControllerOPLin(PendulumParams<T> params):PendulumControllerBase<T>(params) 
{
}



template<typename T>
T PendulumControllerOPLin<T>::compute_controls() const
{
}




