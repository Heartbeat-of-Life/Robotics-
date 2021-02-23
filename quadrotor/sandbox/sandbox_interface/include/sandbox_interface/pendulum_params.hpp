#ifndef PENDULUM_PARAMS
#define PENDULUM_PARAMS
template<typename T>                                                                                             
struct PendulumParams                                                                                          
{                                                                                                                
	public:
	PendulumParams()=default;
        T mw; //wagon mass                                                                                  
	T ms; //stick mass
        T mh; //pendulum head mass                                                                                 
	T xmin;//rail lower bound
	T xmax;//rail upper bound
	T J; //stick inertia w.r.t to the CoM
	T g;//gravity constant
	T l;//length
};
#endif
