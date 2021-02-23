#ifndef PENDULUM_OP_STAB_CONTROLLER
#define PENDULUM_OP_STAB_CONTROLLER
#include "eigen3/Eigen/Dense"
#include "sandbox_interface/pendulum_params.hpp"
#include <cmath>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Core/MatrixBase.h>
#include <eigen3/Eigen/src/Core/util/ForwardDeclarations.h>
#include<iostream>
#include <memory>
#include <string>
#include<grampc.hpp>




#define COS(a) std::cos(a)
#define SIN(a) std::sin(a)
#define POW2(a) ((a)*(a))

template<typename T>
struct Control
{
	T t = -1;
	T u;
};



template<typename T>
class PendulumControllerBase 
{
	public:
	using state_type = Eigen::Matrix<T,4,1>;
	PendulumControllerBase() = delete;
	PendulumControllerBase(const PendulumParams<T> pendulum_params):pendulum_params(pendulum_params)
	{
	}
	virtual std::vector<Control<T>> compute_controls() const = 0;
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
	void set_gravity_constant(const T& g)
	{
		pendulum_params.g=g;
	}
	void set_pendulum_length(const T&l)
	{
		pendulum_params.l=l;
	}
	virtual void init()=0;
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

	void init() override
	{
		const auto & p = this->pendulum_params;
		A = decltype(A)::Zero();
		A(0,1)=1;
		const double a23= p.mh*p.g/p.mw;
		A(1,2)=a23;
		A(2,3)=1;
		A(3,2)=(p.mw+p.mh)*p.g/(p.l*p.mw);
		B(0)=0;
		B(1)=1/p.mw;
		B(2)=0;
		B(3)=1/(p.mw*p.l);
		std::cout <<"A="<<std::endl;
		std::cout <<A<<std::endl;
		std::cout <<"B="<<std::endl;
		std::cout <<B<<std::endl;
		std::cout<<"A Eigenvals="<<std::endl;
		std::cout <<A.eigenvalues()<<std::endl;
	}

	std::vector<Control<T>> compute_controls() const override
	{
		const auto & p = this->pendulum_params;
		auto x = this->state;
		Eigen::Matrix<T,1,4> k;
		k(0) =-4.893; 
		k(1) =-10.1937; 
		k(2) =104.3230; 
		k(3) =30.1937;
		x(0)-=1;
		const auto mat = (A-B*k);
		std::cout<<"mat vals="<<std::endl;
		std::cout <<mat.eigenvalues()<<std::endl;
		Control<T> control;
		control.u= (-k*x);
		return std::vector<Control<T>>(1,control);
	}


	Eigen::Matrix<T,4,4> A;
	Eigen::Matrix<T,4,1> B;
};




template<typename T>
class ProblemDescriptionMPC : public grampc::ProblemDescription
{
	public:
	ProblemDescriptionMPC(const PendulumParams<T> &pendulum_params)
	{
		const auto & p = pendulum_params;
		this->a = p.mw + p.mh + p.ms;
		this->b = -p.l/2*(p.ms+2*p.mh);
		this->c = p.l/2*(p.ms+2*p.mh);
		this->d= p.J  + p.l*p.l*(p.ms/4+p.mh);
		this->e = -p.l/2*(p.ms+2*p.mh);
		this->f = -p.l*p.mh*9.81;
	}
	void ocp_dim(typeInt *Nx, typeInt *Nu, typeInt *Np, typeInt *Ng, typeInt *Nh, typeInt *NgT, typeInt *NhT)override
	{
		*Nx=4;
		*Nu=1;
		*Np=0;
		*Ng=0;
		*Nh=0;
		*NgT=4;
		*NhT=0;

	}
 void Vfct(typeRNum *out, ctypeRNum thor, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes)override 
 {
	 out[0]=   w1_term*POW2(x[0]-xdes[0]);
	 out[0]+=  w2_term*POW2(x[1]-xdes[1]);
	 out[0]+=  w3_term*POW2(x[2]-xdes[2]);
	 out[0]+=  w4_term*POW2(x[3]-xdes[3]);
 }
 virtual void dVdx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes) 
 {
	 out[0]=2*(x[0]-xdes[0])*w1_term;
	 out[1]=2*(x[1]-xdes[1])*w2_term;
	 out[2]=2*(x[2]-xdes[2])*w3_term;
	 out[3]=2*(x[3]-xdes[3])*w4_term;
 }

 void dVdT(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes)
 {
 }


  void gTfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p)override
  {
	out[0]=x[0]-0.1;
	out[1]=x[1];
	out[2]=x[2];
	out[3]=x[3];
  }

    void dgTdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *adj) override
   {
	   out[0]=adj[0];
	   out[1]=adj[1];
	   out[2]=adj[2];
	   out[3]=adj[3];
   }

         void ffct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p) override
	{
		const T a = this->a;
		const T b = this->b;
		const T c = this->c;
		const T d = this->d;
		const T e = this->e;
		const T f = this->f;
		const T phi = x[2];
		const T dphi = x[3];
		const double xddot=(u[0]*d+f*b*SIN(phi)*COS(phi)-d*c*POW2(dphi)*SIN(phi))/(d*a-b*e*POW2(COS(phi)));
		out[0] = x[1];
		out[1] = xddot;
		out[2]=x[3];
		out[3]=-f/d*SIN(phi)-e/d*COS(phi)*xddot;
//		std::cout <<"(-f/d)="<<(-f/d)<<" -e/d="<<(-e/d)<<" COS(phi)="<<COS(phi)<<" SIN(phi)="<<SIN(phi)<<"out[3]="<<out[3]<<" xddot="<<xddot<<std::endl;
////		std::cout <<"ffct input:t="<<t<<", x=["<<x[0]<<","<<x[1]<<","<<x[2]<<","<<x[3]<<"]; u=["<<u[0]<<"]"<<" SIN(phi)="<<SIN(phi)<<std::endl;
//		std::cout <<"prdn input t="<<t<<", x=["<<(x[0]+out[0]*0.2)<<","<<(x[1]+out[1]*0.2)<<","<<(x[2]+out[2]*0.2)<<","<<x[3]+out[3]*0.2<<"]; u=["<<u[0]<<"]"<<std::endl;
//		std::cout<<"[out original=["<<out[0]<<","<<out[1]<<","<<out[2]<<","<<out[3]<<"];"<<std::endl;
//	std::cout <<"output: out=["<<out[0]<<","<<out[1]<<","<<out[2]<<","<<out[3]<<"]; u=["<<u[0]<<"]"<<std::endl;
//		out[0] = x[1];
//		out[1] = x[2]*9.81 + u[0];
//		out[2] = x[3];
//		std::cout<<"diff last="<<(out[3]-x[2]*2*9.81-u[0])<<std::endl;
//		out[3] = x[2]*2*9.81 + u[0];
//		std::cout<<"[out linearized=["<<out[0]<<","<<out[1]<<","<<out[2]<<","<<out[3]<<"];"<<std::endl;
	}


//   virtual void Vfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes) //   {
//	   out[0]=(x[0]-xdes[0])*(x[0]-xdes[0])+(x[1]-xdes[1])*(x[1]-xdes[1]);
//   }
     void lfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes) 
    {
		out[0]= wu_integr*POW2(u[0]);
		out[0]+=w1_integr*POW2(x[0]-xdes[0]);
		out[0]+=w2_integr*POW2(x[1]-xdes[1]);
		out[0]+=w3_integr*POW2(x[2]-xdes[2]);
		out[0]+=w4_integr*POW2(x[3]-xdes[3]);
    }
    /** Gradient dl/dx **/
    void dldx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes)
   {
	   out[0]=2*w1_integr*(x[0]-xdes[0]);
	   out[1]=2*w2_integr*(x[1]-xdes[1]);
	   out[2]=2*w3_integr*(x[2]-xdes[2]);
	   out[3]=2*w4_integr*(x[3]-xdes[3]);
   }
    /** Gradient dl/du **/
     void dldu(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes)
    {
	   out[0]=2*wu_integr*u[0]; 
    }



 void dfdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *adj, ctypeRNum *u, ctypeRNum *p)override
	{
		const T a = this->a;
		const T b = this->b;
		const T c = this->c;
		const T d = this->d;
		const T e = this->e;
		const T f = this->f;
		const double phi=x[2];
		const double dphi=x[3];
		const double denom=(d*a-b*e*POW2(COS(phi)));
//		out[0] = 0;
//		out[1] = 1/denom*adj[0];
//		out[2] = 0;
//		out[3] = COS(phi)/denom*adj[0];


		out[0] = d/denom*adj[1]-e*COS(phi)/denom*adj[3];
//		std::cout <<"dfdu_vec input: x=["<<x[0]<<","<<x[1]<<","<<x[2]<<","<<x[3]<<"]; u=["<<u[0]<<"]"<<std::endl;
//		std::cout<<"[out original=["<<out[0]<<std::endl;
//		out[0] = adj[1]+adj[3];
	}

 
 void dfdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *adj, ctypeRNum *u, ctypeRNum *p)override
	{
		const T a = this->a;
		const T b = this->b;
		const T c = this->c;
		const T d = this->d;
		const T e = this->e;
		const T f = this->f;
		const double phi=x[2];
		const double dphi=x[3];
		const double denom=(d*a-b*e*POW2(COS(phi)));
		out[0]=0;
		out[1]=adj[0];
//		const double xddot=(u[0]*d+f*b*SIN(phi)*COS(phi)-d*c*dphi*dphi*SIN(phi))/(d*a-b*e*COS(phi));

		const double xddot=(u[0]*d+f*b*SIN(phi)*COS(phi)-d*c*POW2(dphi)*SIN(phi))/(d*a-b*e*POW2(COS(phi)));
		const double dxddot_dphi=(((f*b*(POW2(COS(phi))-POW2(SIN(phi)))-d*c*POW2(dphi)*COS(phi))*denom)-(d*u[0]+f*b*SIN(phi)*COS(phi)-d*c*POW2(dphi)*SIN(phi))*2*COS(phi)*b*e*SIN(phi))/POW2(denom);
		const double dphiddot_dphi=-f/d*COS(phi)+e/d*SIN(phi)*xddot-e/d*COS(phi)*dxddot_dphi;
		out[2]=dxddot_dphi*adj[1]+dphiddot_dphi*adj[3];
		out[3]=-2*d*c*dphi*SIN(phi)/denom*adj[1]+e/d*COS(phi)*(d*c*2*dphi*SIN(phi))/denom*adj[3]+adj[2];
//	std::cout<<"t="<<t<<" x=["<<x[0]<<" "<<x[1]<<" "<<x[2]<<" "<<x[3]<<"];"<<" u="<<u[0]<<" cos(phi)="<<COS(phi)<<" SIN(PHI)="<<SIN(phi) <<std::endl;
//	std::cout <<"xddot="<<xddot<<" dxddot="<<dxddot_dphi<<" (fb/(da-be)) "<< (f*b/(d*a-b*e)) <<" dphiddot_dphi="<<dphiddot_dphi<<" dxddot/ddphi="<<( -2*d*c*dphi*SIN(phi)/denom )<<"  dphiddot/ddothpi="<<(   e/d*COS(phi)*(-d*c*2*dphi*SIN(phi))/denom)  << std::endl;
//		out[0] = 0;
//		out[1]= adj[0];
//		out[2]=9.81*adj[1]+2*9.81*adj[3];
//		out[3]=adj[2];
//		std::cout <<"dfdx_vec input: x=["<<x[0]<<","<<x[1]<<","<<x[2]<<","<<x[3]<<"]; u=["<<u[0]<<"]"<<std::endl;
//		std::cout<<"[out original=["<<out[0]<<","<<out[1]<<","<<out[2]<<","<<out[3]<<"];"<<std::endl;
	}
	T a, b, c, d, e, f;




	 double w1_integr;
	 double w2_integr;
	 double w3_integr;
	 double w4_integr;
	 double wu_integr;

	 double w1_term;
	 double w2_term;
	 double w3_term;
	 double w4_term;

	 double x1f;
	 double x2f;
	 double x3f;
	 double x4f;


	void setTerminalCostWeights(const double w1,
		       	   	    const double w2, 
				    const double w3,
				    const double w4)
	{
		w1_term=w1;
		w2_term=w2;
		w3_term=w3;
		w4_term=w4;
	}
	void setIntegralCostWeights(const double w1,
		       		const double w2, 
				const double w3,
				const double w4,
				const double wu)
	{
		w1_integr=w1;
		w2_integr=w2;
		w3_integr=w3;
		w4_integr=w4;
		wu_integr=wu;
	}
	void setTerminalConstraints(const double x1f_,
				    const double x2f_,
				    const double x3f_,
				    const double x4f_)
	{
		x1f=x1f_;
		x2f=x2f_;
		x3f=x3f_;
		x4f=x4f_;
	}

};

template<typename T>
class PendulumControllerMPC : public PendulumControllerBase<T>
{

      public:
      PendulumControllerMPC(const PendulumParams<T> pendulum_params):PendulumControllerBase<T>(pendulum_params)
      {
	      prob_form= std::make_shared<ProblemDescriptionMPC<T>>(pendulum_params);
 	      solver = std::make_unique<grampc::Grampc>(prob_form.get());


      }



	void init() override
	{
	}




	std::vector<Control<T>> compute_controls() const override
	{
//	FILE *file_x, *file_u, *file_p, *file_T, *file_J, *file_Ncfct, *file_Npen, *file_iter, *file_status, *file_t;

//           openFile(&file_x, "res/xvec.txt");
//           openFile(&file_u, "res/uvec.txt");
//           openFile(&file_p, "res/pvec.txt");
//           openFile(&file_T, "res/Thorvec.txt");
//           openFile(&file_J, "res/Jvec.txt");
//           openFile(&file_Ncfct, "res/Ncfctvec.txt");
//           openFile(&file_Npen, "res/Npenvec.txt");
//           openFile(&file_iter, "res/itervec.txt");
//           openFile(&file_status, "res/status.txt");
//           openFile(&file_t, "res/tvec.txt");


//		solver.setTerminalCostWeights(0,0,0,0);
//		solver.setIntegralCostWeights(0,0,0,0,1);

		const int N = 1001;
		typeRNum Thor = 5;
		// Order is x1 x2 x3 x4 u
		prob_form->setTerminalCostWeights(0,0,0,0);
		prob_form->setIntegralCostWeights(0,0,0,0,1);
		prob_form->setTerminalConstraints(0,0,0,0);

		ctypeRNum constraintstols[4] = {1e-3,1e-3,1e-3,1e-3};
//		a;
		const int max_mult_iter=10;
		const int max_grad_iter=15;
		ctypeRNum xdes[4] = {0,0,0,0};
		ctypeRNum x0[4] = {0,0,0.1,0};
		
//		ctypeRNum x0[4] = {};

//		ctypeRNum x0[4] = {this->state(0),this->state(1),this->state(2),this->state(3)};
//		std::cout<<"x09=["<<x0[0]<<" "<<x0[1]<<" "<<x0[2]<<" "<<x0[3]<<std::endl;
		ctypeRNum u0[1] = {1};
//		std::cout<<"x0="<<x0[0]<< " "<<x0[1]<<" "<<x0[2]<< " "<<x0[3]<<std::endl;
		solver->setparam_real_vector("x0",x0);
//		solver->setparam_real_vector("u0",u0);
		solver->setparam_real_vector("xdes",xdes);
		solver->setopt_real_vector("ConstraintsAbsTol",constraintstols);
		solver->setopt_int("Nhor",N);
//		solver->setparam_real("t0",0);
		solver->setparam_real("dt",10e-3);
		solver->setopt_real("ConvergenceGradientRelTol",1e-8);
		solver->setopt_string("ShiftControl","off");
		solver->setopt_string("ConvergenceCheck","on");
		solver->setopt_string("TerminalInequalityConstraints","off");
		solver->setopt_string("TerminalEqualityConstraints","on");
		solver->setopt_string("EqualityConstraints","off");
		solver->setopt_string("InequalityConstraints","off");
		solver->setopt_string("TerminalCost","off");
		solver->setopt_string("OptimTime","off");
		solver->setopt_string("Integrator","heun");
//		solver->setopt_string("ScaleProblem","on");
		solver->setopt_int("MaxMultIter",max_mult_iter);
		solver->setopt_int("MaxGradIter",max_grad_iter);
		solver->setparam_real("Thor",Thor);
		solver->printopt();
		ctypeRNum umax[1] = {100};
		ctypeRNum umin[1] = {-100};
		solver->setparam_real_vector("umax",umax);
		solver->setparam_real_vector("umin",umin);
		solver->run(); 
		const auto wsp = solver->getWorkspace();
		std::vector<Control<T>> res(N);
		std::cout<<"NEW RESULT="<<std::endl;
		for(size_t k = 0; k < N; ++k)
		{
			res[k].t=wsp->t[k];
			res[k].u=wsp->u[k];
//		std::cout <<"u="<<wsp->u[k]<<" x?="<<wsp->x[2*k]<<" "<<wsp->x[2*k+1]<<std::endl;
		std::cout <<"t="<<res[k].t<<  " u="<<wsp->u[k]<<" x= "<<wsp->x[4*k]<< " "<<wsp->x[4*k+1]<<" "<<wsp->x[4*k+2]<<" "<<wsp->x[4*k+3]<<std::endl;
	}
//		std::cout <<"FINAL COSTS="<<(solver->getSolution()->cfct)<<std::endl;
		for(size_t k=0; k <max_mult_iter; ++k)
		{
			std::cout <<" maxgraditer[k="<<k<<"]="<<(solver->getSolution()->iter[k])<<" J="<<(solver->getSolution()->J[k])<<std::endl;
		}




		std::cout<<"status...DEBUG"<<std::endl;
		solver->printstatus(solver->getSolution()->status,STATUS_LEVEL_DEBUG);
		std::cout<<"status...INFO"<<std::endl;
		solver->printstatus(solver->getSolution()->status,STATUS_LEVEL_INFO);
		std::cout<<"status... WARN"<<std::endl;
		solver->printstatus(solver->getSolution()->status,STATUS_LEVEL_WARN);
//		solver->printstatus(solver->getSolution()->status,STATUS_INFEASIBLE);

		return res;

	}
	std::unique_ptr<grampc::Grampc> solver;//
	std::shared_ptr<ProblemDescriptionMPC<T>> prob_form;
	                                       //
                                               //
};









#endif
