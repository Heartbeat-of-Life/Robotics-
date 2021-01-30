#ifndef PENDULUM_PARAMS
#define PENDULUM_PARAMS
template<typename T>                                                                                             
struct PendulumParams                                                                                          
{                                                                                                                
	public:
	PendulumParams()=default;
        T mw; //wagon mass                                                                                  
        T mh; //pendulum head mass                                                                                 
	T xmin;
	T xmax;
};
#endif
