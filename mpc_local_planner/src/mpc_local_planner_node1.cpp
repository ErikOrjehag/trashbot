
// states: px, py, steer, v
// control: steer_w, accel

#include "psopt.h"

//////////////////////////////////////////////////////////////////////////
///////////////////  Define the end point (Mayer) cost function //////////
//////////////////////////////////////////////////////////////////////////


adouble endpoint_cost(adouble* initial_states, adouble* final_states,
                      adouble* parameters, adouble& t0, adouble& tf,
                      adouble* xad, int iphase, Workspace* workspace)
{
   //return pow(final_states[ CINDEX(1) ] - 1.0, 2) + pow(final_states[ CINDEX(2) ] - 0.5, 2);
   return 0;
}

//////////////////////////////////////////////////////////////////////////
///////////////////  Define the integrand (Lagrange) cost function  //////
//////////////////////////////////////////////////////////////////////////

adouble integrand_cost(adouble* states, adouble* controls, adouble* parameters,
                     adouble& time, adouble* xad, int iphase, Workspace* workspace)
{
  adouble steer_w = controls[ CINDEX(1) ];
  adouble accel   = controls[ CINDEX(2) ];

   //return pow(steer_w, 2) + pow(accel, 2);
   return 0;
}


//////////////////////////////////////////////////////////////////////////
///////////////////  Define the DAE's ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////


void dae(adouble* derivatives, adouble* path, adouble* states,
         adouble* controls, adouble* parameters, adouble& time,
         adouble* xad, int iphase, Workspace* workspace)
{

    adouble steer_w 	= controls[ CINDEX(1) ];
    adouble accel   	= controls[ CINDEX(2) ];

    adouble px 	    = states[ CINDEX(1) ];
    adouble py 	    = states[ CINDEX(2) ];
    adouble steer 	= states[ CINDEX(3) ];
    adouble v 	    = states[ CINDEX(4) ];

    adouble d_px 	    = v * cos(steer);
    adouble d_py 	    = v * sin(steer);
    adouble d_steer 	= steer_w;
    adouble d_v 	    = accel;

    derivatives[ CINDEX(1) ] = d_px;
    derivatives[ CINDEX(2) ] = d_py;
    derivatives[ CINDEX(3) ] = d_steer;
    derivatives[ CINDEX(4) ] = d_v;
}

////////////////////////////////////////////////////////////////////////////
///////////////////  Define the events function ////////////////////////////
////////////////////////////////////////////////////////////////////////////

void events(
  adouble* e,
  adouble* initial_states,
  adouble* final_states,
  adouble* parameters,
  adouble& t0,
  adouble& tf,
  adouble* xad,
  int iphase,
  Workspace* workspace
)
{
    adouble px_0 	    = initial_states[ CINDEX(1) ];
    adouble py_0 	    = initial_states[ CINDEX(2) ];
    adouble steer_0 	= initial_states[ CINDEX(3) ];
    adouble v_0 	    = initial_states[ CINDEX(4) ];

    adouble px_f 	    = final_states[ CINDEX(1) ];
    adouble py_f 	    = final_states[ CINDEX(2) ];
    adouble steer_f 	= final_states[ CINDEX(3) ];
    adouble v_f 	    = final_states[ CINDEX(4) ];

    e[ CINDEX(1) ] = px_0;
    e[ CINDEX(2) ] = py_0;
    e[ CINDEX(3) ] = steer_0;
    e[ CINDEX(4) ] = v_0;

    e[ CINDEX(5) ] = px_f;
    e[ CINDEX(6) ] = py_f;
    e[ CINDEX(7) ] = steer_f;
    e[ CINDEX(8) ] = v_f;
}



///////////////////////////////////////////////////////////////////////////
///////////////////  Define the phase linkages function ///////////////////
///////////////////////////////////////////////////////////////////////////

void linkages( adouble* linkages, adouble* xad, Workspace* workspace)
{

   // Single phase problem

}



////////////////////////////////////////////////////////////////////////////
///////////////////  Define the main routine ///////////////////////////////
////////////////////////////////////////////////////////////////////////////

int main(void)
{

////////////////////////////////////////////////////////////////////////////
///////////////////  Declare key structures ////////////////////////////////
////////////////////////////////////////////////////////////////////////////

    Alg  algorithm;
    Sol  solution;
    Prob problem;

////////////////////////////////////////////////////////////////////////////
///////////////////  Register problem name  ////////////////////////////////
////////////////////////////////////////////////////////////////////////////

    problem.name        		= "Trashbot problem";
    problem.outfilename     = "trashbot.txt";

////////////////////////////////////////////////////////////////////////////
////////////  Define problem level constants & do level 1 setup ////////////
////////////////////////////////////////////////////////////////////////////

    problem.nphases = 1;
    problem.nlinkages = 0;

    psopt_level1_setup(problem);

/////////////////////////////////////////////////////////////////////////////
/////////   Define phase related information & do level 2 setup  ////////////
/////////////////////////////////////////////////////////////////////////////

    problem.phases(1).nstates   = 4;
    problem.phases(1).ncontrols = 2;
    problem.phases(1).nevents   = 8;
    problem.phases(1).npath     = 0;
    problem.phases(1).nodes     = "[30]"; //30 40 50 80

    psopt_level2_setup(problem, algorithm);

////////////////////////////////////////////////////////////////////////////
///////////////////  Enter problem bounds information //////////////////////
////////////////////////////////////////////////////////////////////////////

    problem.phases(1).bounds.lower.states(1) = -1e6; // px
    problem.phases(1).bounds.upper.states(1) =  1e6;

    problem.phases(1).bounds.lower.states(2) = -1e6; // py
    problem.phases(1).bounds.upper.states(2) =  1e6;

    problem.phases(1).bounds.lower.states(3) = -1e6; // steer
    problem.phases(1).bounds.upper.states(3) =  1e6;

    problem.phases(1).bounds.lower.states(4) = -0.5; // v
    problem.phases(1).bounds.upper.states(4) =  0.5;

    problem.phases(1).bounds.lower.controls(1) = -0.1; // steer_w
    problem.phases(1).bounds.upper.controls(1) = 0.1;

    problem.phases(1).bounds.lower.controls(2) = -0.1; // accel
    problem.phases(1).bounds.upper.controls(2) = 0.1;

    problem.phases(1).bounds.lower.events(1) = 0.0; // px_0
    problem.phases(1).bounds.upper.events(1) = problem.phases(1).bounds.lower.events(1);
    problem.phases(1).bounds.lower.events(2) = 0.0; // py_0
    problem.phases(1).bounds.upper.events(2) = problem.phases(1).bounds.lower.events(2);
    problem.phases(1).bounds.lower.events(3) = 0.0; // steer_0
    problem.phases(1).bounds.upper.events(3) = problem.phases(1).bounds.lower.events(3);
    problem.phases(1).bounds.lower.events(4) = 0.0; // v_0
    problem.phases(1).bounds.upper.events(4) = problem.phases(1).bounds.lower.events(4);

    problem.phases(1).bounds.lower.events(5) = 1.0; // px_f
    problem.phases(1).bounds.upper.events(5) = problem.phases(1).bounds.lower.events(5);
    problem.phases(1).bounds.lower.events(6) = 0.5; // py_f
    problem.phases(1).bounds.upper.events(6) = problem.phases(1).bounds.lower.events(6);
    problem.phases(1).bounds.lower.events(7) = 0.0; // steer_f
    problem.phases(1).bounds.upper.events(7) = problem.phases(1).bounds.lower.events(7);
    problem.phases(1).bounds.lower.events(8) = 0.0; // v_f
    problem.phases(1).bounds.upper.events(8) = problem.phases(1).bounds.lower.events(8);

    problem.phases(1).bounds.lower.StartTime = 0.0;
    problem.phases(1).bounds.upper.StartTime = problem.phases(1).bounds.upper.StartTime;

    problem.phases(1).bounds.lower.EndTime      = 0.0;
    problem.phases(1).bounds.upper.EndTime      = 10.0;

////////////////////////////////////////////////////////////////////////////
///////////////////  Register problem functions  ///////////////////////////
////////////////////////////////////////////////////////////////////////////

    problem.integrand_cost = &integrand_cost;
    problem.endpoint_cost  = &endpoint_cost;
    problem.dae            = &dae;
    problem.events 		     = &events;
    problem.linkages		   = &linkages;

////////////////////////////////////////////////////////////////////////////
///////////////////  Define & register initial guess ///////////////////////
////////////////////////////////////////////////////////////////////////////

    int nnodes    			   = problem.phases(1).nodes(1);
    int ncontrols          = problem.phases(1).ncontrols;
    int nstates            = problem.phases(1).nstates;

    DMatrix state_guess    =  zeros(nstates, nnodes);
    DMatrix control_guess  =  0.0 * ones(ncontrols, nnodes);
    DMatrix time_guess     =  linspace(0.0, 5.0, nnodes);

    state_guess(1, colon()) = linspace(0.0, 1.0, nnodes);
    state_guess(2, colon()) = linspace(0.0, 0.5, nnodes);

    problem.phases(1).guess.states   = state_guess;
    problem.phases(1).guess.controls = control_guess;
    problem.phases(1).guess.time     = time_guess;

////////////////////////////////////////////////////////////////////////////
///////////////////  Enter algorithm options  //////////////////////////////
////////////////////////////////////////////////////////////////////////////

    algorithm.nlp_iter_max                = 1000; //= 1000;
    algorithm.nlp_tolerance               = 1.e-6; //= 1.e-6;
    algorithm.nlp_method                  = "IPOPT";
    algorithm.scaling                     = "automatic";
    algorithm.derivatives                 = "automatic";
    algorithm.collocation_method          = "Legendre";

////////////////////////////////////////////////////////////////////////////
///////////////////  Now call PSOPT to solve the problem   //////////////////
////////////////////////////////////////////////////////////////////////////

    psopt(solution, problem, algorithm);

////////////////////////////////////////////////////////////////////////////
///////////  Extract relevant variables from solution structure   //////////
////////////////////////////////////////////////////////////////////////////

    DMatrix x, u, t;

    x = solution.get_states_in_phase(1);
    u = solution.get_controls_in_phase(1);
    t = solution.get_time_in_phase(1);

    x.Save("x.dat");
    u.Save("u.dat");
    t.Save("t.dat");
}










/*
#include "psopt.h"

adouble endpoint_cost(adouble* initial_states, adouble* final_states,
                      adouble* parameters,adouble& t0, adouble& tf,
                      adouble* xad, int iphase, Workspace* workspace)
{
  return 0;
}

adouble integrand_cost(adouble* states, adouble* controls, adouble* parameters,
                     adouble& time, adouble* xad, int iphase, Workspace* workspace)
{
  return 0.0;
}

void dae(adouble* derivatives, adouble* path, adouble* states,
         adouble* controls, adouble* parameters, adouble& time,
         adouble* xad, int iphase, Workspace* workspace)
{
  adouble CL 		= controls[ CINDEX(1) ];
  adouble vx 		= states[ CINDEX(3) ];
  adouble vy 	    = states[ CINDEX(4) ];
  vr = sqrt(vx*vx + vy*vy);
  derivatives[ CINDEX(1) ] =   vr;
}

void events(adouble* e, adouble* initial_states, adouble* final_states,
            adouble* parameters, adouble& t0, adouble& tf, adouble* xad,
            int iphase, Workspace* workspace)

{

}

void linkages( adouble* linkages, adouble* xad, Workspace* workspace)
{

}

int main(void)
{
  Alg  algorithm;
  Sol  solution;
  Prob problem;

  problem.name = "mpc_local_planner";

  problem.nphases = 1;
  problem.nlinkages = 0;

  psopt_level1_setup(problem);

  problem.phases(1).nstates = 4;
  problem.phases(1).ncontrols = 1;
  problem.phases(1).nevents = 7;
  problem.phases(1).npath = 0;
  problem.phases(1).nodes = "[30 40 50 80]";

  psopt_level2_setup(problem, algorithm);

  problem.phases(1).bounds.lower.states = "[0.0      0.0    0.0  -4.0]";
  problem.phases(1).bounds.upper.states = "[1500.0  1100.0 15.0   4.0]";


  problem.phases(1).bounds.lower.controls(1) = 0.0;
  problem.phases(1).bounds.upper.controls(1) = 1.4;


  problem.phases(1).bounds.lower.events = "[0.0,1000.0,13.2275675,-1.28750052,900.00,13.2275675,-1.28750052 ]";
  problem.phases(1).bounds.upper.events = "[0.0,1000.0,13.2275675,-1.28750052,900.00,13.2275675,-1.28750052 ]";

  problem.phases(1).bounds.lower.StartTime = 0.0;
  problem.phases(1).bounds.upper.StartTime = 0.0;

  problem.phases(1).bounds.lower.EndTime = 2.0;
  problem.phases(1).bounds.upper.EndTime = 2.0;

  problem.integrand_cost = &integrand_cost;
  problem.endpoint_cost = &endpoint_cost;
  problem.dae = &dae;
  problem.events = &events;
  problem.linkages = &linkages;

  // Initial guess

  int nnodes = problem.phases(1).nodes(1);
  int ncontrols = problem.phases(1).ncontrols;
  int nstates = problem.phases(1).nstates;

  DMatrix state_guess = zeros(nstates, nnodes);
  DMatrix control_guess = 1.0 * ones(ncontrols, nnodes);
  DMatrix time_guess = linspace(0.0,105.0, nnodes);

  state_guess(1, colon()) = linspace(0.0, 1250, nnodes);
  state_guess(2, colon()) = linspace(1000.0, 900.0, nnodes);
  state_guess(3, colon()) = 13.23 * ones(1, nnodes);
  state_guess(4, colon()) = -1.288 * ones(1, nnodes);

  problem.phases(1).guess.states = state_guess;
  problem.phases(1).guess.controls = control_guess;
  problem.phases(1).guess.time = time_guess;

  // Solver settings

  algorithm.nlp_iter_max                = 1000;
  algorithm.nlp_tolerance               = 1.e-6;
  algorithm.nlp_method                  = "IPOPT";
  algorithm.scaling                     = "automatic";
  algorithm.derivatives                 = "automatic";

  // Solve the problem!
  psopt(solution, problem, algorithm);
}
*/
