
// states: x, v
// control: a

#include "psopt.h"

//////////////////////////////////////////////////////////////////////////
///////////////////  Define the end point (Mayer) cost function //////////
//////////////////////////////////////////////////////////////////////////


adouble endpoint_cost(adouble* initial_states, adouble* final_states,
                      adouble* parameters, adouble& t0, adouble& tf,
                      adouble* xad, int iphase, Workspace* workspace)
{
  adouble x = final_states[ CINDEX(1) ];
  adouble v = final_states[ CINDEX(2) ];

  //return 0;
  return pow(x - 0.2, 2);
}

//////////////////////////////////////////////////////////////////////////
///////////////////  Define the integrand (Lagrange) cost function  //////
//////////////////////////////////////////////////////////////////////////

adouble integrand_cost(adouble* states, adouble* controls, adouble* parameters,
                     adouble& time, adouble* xad, int iphase, Workspace* workspace)
{
  adouble x = states[ CINDEX(1) ];
  adouble v = states[ CINDEX(2) ];
  return 1;
  //return pow(v - 0.3, 2);
  //return pow(x - 0.2, 2) + pow(v-10, 2);
  //return 0;
  //return pow(controls[0], 2);
  //return pow(controls[ CINDEX(1) ], 2);
}


//////////////////////////////////////////////////////////////////////////
///////////////////  Define the DAE's ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////


void dae(adouble* derivatives, adouble* path, adouble* states,
         adouble* controls, adouble* parameters, adouble& time,
         adouble* xad, int iphase, Workspace* workspace)
{
    adouble x = states[ CINDEX(1) ];
    adouble v = states[ CINDEX(2) ];

    derivatives[ CINDEX(1) ] = v;
    derivatives[ CINDEX(2) ] = controls[ CINDEX(1) ];
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
    e[ CINDEX(1) ] = initial_states[ CINDEX(1) ];
    e[ CINDEX(2) ] = initial_states[ CINDEX(2) ];
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

    problem.phases(1).nstates   = 2;
    problem.phases(1).ncontrols = 1;
    problem.phases(1).nevents   = 2;
    problem.phases(1).npath     = 0;
    problem.phases(1).nodes     = 5; //30 40 50 80

    psopt_level2_setup(problem, algorithm);

////////////////////////////////////////////////////////////////////////////
///////////////////  Enter problem bounds information //////////////////////
////////////////////////////////////////////////////////////////////////////

    problem.phases(1).bounds.lower.states(1) = -10; // x
    problem.phases(1).bounds.upper.states(1) =  10;

    problem.phases(1).bounds.lower.states(2) = -0.3; // v
    problem.phases(1).bounds.upper.states(2) =  0.3;

    problem.phases(1).bounds.lower.controls(1) = -3.0; // a
    problem.phases(1).bounds.upper.controls(1) = 3.0;

    problem.phases(1).bounds.lower.events(1) = 0.0; // x_0
    problem.phases(1).bounds.upper.events(1) = problem.phases(1).bounds.lower.events(1);

    problem.phases(1).bounds.lower.events(2) = 0.0; // v_0
    problem.phases(1).bounds.upper.events(2) = problem.phases(1).bounds.lower.events(2);

    problem.phases(1).bounds.lower.StartTime = 0.0;
    problem.phases(1).bounds.upper.StartTime = problem.phases(1).bounds.upper.StartTime;

    problem.phases(1).bounds.lower.EndTime = 2.0;
    problem.phases(1).bounds.upper.EndTime = 2.0;

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
    DMatrix control_guess  =  0.5 * ones(ncontrols, nnodes);
    DMatrix time_guess     =  linspace(0.0, 1.0, nnodes);

    state_guess(1, colon()) = linspace(0.0, 0.2, nnodes);

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
