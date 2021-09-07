/*
 * Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
 * Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
 * Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
 * Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

// example specific
#include "quadrotor_q_model/quadrotor_q_model.h"





#include "acados_solver_quadrotor_q.h"

#define NX     10
#define NZ     0
#define NU     4
#define NP     0
#define NBX    0
#define NBX0   10
#define NBU    4
#define NSBX   0
#define NSBU   0
#define NSH    0
#define NSG    0
#define NSPHI  0
#define NSHN   0
#define NSGN   0
#define NSPHIN 0
#define NSBXN  0
#define NS     0
#define NSN    0
#define NG     0
#define NBXN   0
#define NGN    0
#define NY0    14
#define NY     14
#define NYN    6
#define N      20
#define NH     0
#define NPHI   0
#define NHN    0
#define NPHIN  0
#define NR     0


// ** solver data **

nlp_solver_capsule * quadrotor_q_acados_create_capsule()
{
    void* capsule_mem = malloc(sizeof(nlp_solver_capsule));
    nlp_solver_capsule *capsule = (nlp_solver_capsule *) capsule_mem;

    return capsule;
}


int quadrotor_q_acados_free_capsule(nlp_solver_capsule *capsule)
{
    free(capsule);
    return 0;
}


int quadrotor_q_acados_create(nlp_solver_capsule * capsule)
{
    int status = 0;

    // number of expected runtime parameters
    capsule->nlp_np = NP;

    /************************************************
    *  plan & config
    ************************************************/
    ocp_nlp_plan * nlp_solver_plan = ocp_nlp_plan_create(N);
    capsule->nlp_solver_plan = nlp_solver_plan;
    nlp_solver_plan->nlp_solver = SQP;
    

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = FULL_CONDENSING_HPIPM;

    nlp_solver_plan->nlp_cost[0] = LINEAR_LS;
    for (int i = 1; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = LINEAR_LS;

    nlp_solver_plan->nlp_cost[N] = LINEAR_LS;

    for (int i = 0; i < N; i++)
    {
        
        nlp_solver_plan->nlp_dynamics[i] = CONTINUOUS_MODEL;
        nlp_solver_plan->sim_solver_plan[i].sim_solver = ERK;
    }

    for (int i = 0; i < N; i++)
    {
        nlp_solver_plan->nlp_constraints[i] = BGH;
    }
    nlp_solver_plan->nlp_constraints[N] = BGH;
    ocp_nlp_config * nlp_config = ocp_nlp_config_create(*nlp_solver_plan);
    capsule->nlp_config = nlp_config;


    /************************************************
    *  dimensions
    ************************************************/
    int nx[N+1];
    int nu[N+1];
    int nbx[N+1];
    int nbu[N+1];
    int nsbx[N+1];
    int nsbu[N+1];
    int nsg[N+1];
    int nsh[N+1];
    int nsphi[N+1];
    int ns[N+1];
    int ng[N+1];
    int nh[N+1];
    int nphi[N+1];
    int nz[N+1];
    int ny[N+1];
    int nr[N+1];
    int nbxe[N+1];

    for (int i = 0; i < N+1; i++)
    {
        // common
        nx[i]     = NX;
        nu[i]     = NU;
        nz[i]     = NZ;
        ns[i]     = NS;
        // cost
        ny[i]     = NY;
        // constraints
        nbx[i]    = NBX;
        nbu[i]    = NBU;
        nsbx[i]   = NSBX;
        nsbu[i]   = NSBU;
        nsg[i] = NSG;
        nsh[i]    = NSH;
        nsphi[i]  = NSPHI;
        ng[i]     = NG;
        nh[i]     = NH;
        nphi[i]   = NPHI;
        nr[i]     = NR;
        nbxe[i]   = 0;
    }

    // for initial state
    nbx[0]  = NBX0;
    nsbx[0] = 0;
    ns[0] = NS - NSBX;
    nbxe[0] = 10;
    ny[0] = NY0;

    // terminal - common
    nu[N]   = 0;
    nz[N]   = 0;
    ns[N]   = NSN;
    // cost
    ny[N]   = NYN;
    // constraint
    nbx[N]   = NBXN;
    nbu[N]   = 0;
    ng[N]    = NGN;
    nh[N]    = NHN;
    nphi[N]  = NPHIN;
    nr[N]    = 0;

    nsbx[N]  = NSBXN;
    nsbu[N]  = 0;
    nsg[N]   = NSGN;
    nsh[N]   = NSHN;
    nsphi[N] = NSPHIN;

    /* create and set ocp_nlp_dims */
    ocp_nlp_dims * nlp_dims = ocp_nlp_dims_create(nlp_config);
    capsule->nlp_dims = nlp_dims;

    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nx", nx);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nu", nu);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nz", nz);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "ns", ns);

    for (int i = 0; i <= N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbx", &nbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbu", &nbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbx", &nsbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbu", &nsbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "ng", &ng[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsg", &nsg[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbxe", &nbxe[i]);
    }
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, 0, "ny", &ny[0]);
    for (int i = 1; i < N; i++)
        ocp_nlp_dims_set_cost(nlp_config, nlp_dims, i, "ny", &ny[i]);

    for (int i = 0; i < N; i++)
    {
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nh", &nh[N]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nsh", &nsh[N]);
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, N, "ny", &ny[N]);



    /************************************************
    *  external functions
    ************************************************/


    // explicit ode
    capsule->forw_vde_casadi = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        capsule->forw_vde_casadi[i].casadi_fun = &quadrotor_q_expl_vde_forw;
        capsule->forw_vde_casadi[i].casadi_n_in = &quadrotor_q_expl_vde_forw_n_in;
        capsule->forw_vde_casadi[i].casadi_n_out = &quadrotor_q_expl_vde_forw_n_out;
        capsule->forw_vde_casadi[i].casadi_sparsity_in = &quadrotor_q_expl_vde_forw_sparsity_in;
        capsule->forw_vde_casadi[i].casadi_sparsity_out = &quadrotor_q_expl_vde_forw_sparsity_out;
        capsule->forw_vde_casadi[i].casadi_work = &quadrotor_q_expl_vde_forw_work;
        external_function_param_casadi_create(&capsule->forw_vde_casadi[i], 0);
    }

    capsule->expl_ode_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        capsule->expl_ode_fun[i].casadi_fun = &quadrotor_q_expl_ode_fun;
        capsule->expl_ode_fun[i].casadi_n_in = &quadrotor_q_expl_ode_fun_n_in;
        capsule->expl_ode_fun[i].casadi_n_out = &quadrotor_q_expl_ode_fun_n_out;
        capsule->expl_ode_fun[i].casadi_sparsity_in = &quadrotor_q_expl_ode_fun_sparsity_in;
        capsule->expl_ode_fun[i].casadi_sparsity_out = &quadrotor_q_expl_ode_fun_sparsity_out;
        capsule->expl_ode_fun[i].casadi_work = &quadrotor_q_expl_ode_fun_work;
        external_function_param_casadi_create(&capsule->expl_ode_fun[i], 0);
    }



    /************************************************
    *  nlp_in
    ************************************************/
    ocp_nlp_in * nlp_in = ocp_nlp_in_create(nlp_config, nlp_dims);
    capsule->nlp_in = nlp_in;

    double time_steps[N];
    time_steps[0] = 0.1;
    time_steps[1] = 0.1;
    time_steps[2] = 0.1;
    time_steps[3] = 0.1;
    time_steps[4] = 0.1;
    time_steps[5] = 0.1;
    time_steps[6] = 0.1;
    time_steps[7] = 0.1;
    time_steps[8] = 0.1;
    time_steps[9] = 0.1;
    time_steps[10] = 0.1;
    time_steps[11] = 0.1;
    time_steps[12] = 0.1;
    time_steps[13] = 0.1;
    time_steps[14] = 0.1;
    time_steps[15] = 0.1;
    time_steps[16] = 0.1;
    time_steps[17] = 0.1;
    time_steps[18] = 0.1;
    time_steps[19] = 0.1;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &time_steps[i]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &time_steps[i]);
    }

    /**** Dynamics ****/
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "expl_vde_forw", &capsule->forw_vde_casadi[i]);
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "expl_ode_fun", &capsule->expl_ode_fun[i]);
    
    }


    /**** Cost ****/

    double W_0[NY0*NY0];
    
    W_0[0+(NY0) * 0] = 20;
    W_0[0+(NY0) * 1] = 0;
    W_0[0+(NY0) * 2] = 0;
    W_0[0+(NY0) * 3] = 0;
    W_0[0+(NY0) * 4] = 0;
    W_0[0+(NY0) * 5] = 0;
    W_0[0+(NY0) * 6] = 0;
    W_0[0+(NY0) * 7] = 0;
    W_0[0+(NY0) * 8] = 0;
    W_0[0+(NY0) * 9] = 0;
    W_0[0+(NY0) * 10] = 0;
    W_0[0+(NY0) * 11] = 0;
    W_0[0+(NY0) * 12] = 0;
    W_0[0+(NY0) * 13] = 0;
    W_0[1+(NY0) * 0] = 0;
    W_0[1+(NY0) * 1] = 20;
    W_0[1+(NY0) * 2] = 0;
    W_0[1+(NY0) * 3] = 0;
    W_0[1+(NY0) * 4] = 0;
    W_0[1+(NY0) * 5] = 0;
    W_0[1+(NY0) * 6] = 0;
    W_0[1+(NY0) * 7] = 0;
    W_0[1+(NY0) * 8] = 0;
    W_0[1+(NY0) * 9] = 0;
    W_0[1+(NY0) * 10] = 0;
    W_0[1+(NY0) * 11] = 0;
    W_0[1+(NY0) * 12] = 0;
    W_0[1+(NY0) * 13] = 0;
    W_0[2+(NY0) * 0] = 0;
    W_0[2+(NY0) * 1] = 0;
    W_0[2+(NY0) * 2] = 20;
    W_0[2+(NY0) * 3] = 0;
    W_0[2+(NY0) * 4] = 0;
    W_0[2+(NY0) * 5] = 0;
    W_0[2+(NY0) * 6] = 0;
    W_0[2+(NY0) * 7] = 0;
    W_0[2+(NY0) * 8] = 0;
    W_0[2+(NY0) * 9] = 0;
    W_0[2+(NY0) * 10] = 0;
    W_0[2+(NY0) * 11] = 0;
    W_0[2+(NY0) * 12] = 0;
    W_0[2+(NY0) * 13] = 0;
    W_0[3+(NY0) * 0] = 0;
    W_0[3+(NY0) * 1] = 0;
    W_0[3+(NY0) * 2] = 0;
    W_0[3+(NY0) * 3] = 0.03;
    W_0[3+(NY0) * 4] = 0;
    W_0[3+(NY0) * 5] = 0;
    W_0[3+(NY0) * 6] = 0;
    W_0[3+(NY0) * 7] = 0;
    W_0[3+(NY0) * 8] = 0;
    W_0[3+(NY0) * 9] = 0;
    W_0[3+(NY0) * 10] = 0;
    W_0[3+(NY0) * 11] = 0;
    W_0[3+(NY0) * 12] = 0;
    W_0[3+(NY0) * 13] = 0;
    W_0[4+(NY0) * 0] = 0;
    W_0[4+(NY0) * 1] = 0;
    W_0[4+(NY0) * 2] = 0;
    W_0[4+(NY0) * 3] = 0;
    W_0[4+(NY0) * 4] = 0.03;
    W_0[4+(NY0) * 5] = 0;
    W_0[4+(NY0) * 6] = 0;
    W_0[4+(NY0) * 7] = 0;
    W_0[4+(NY0) * 8] = 0;
    W_0[4+(NY0) * 9] = 0;
    W_0[4+(NY0) * 10] = 0;
    W_0[4+(NY0) * 11] = 0;
    W_0[4+(NY0) * 12] = 0;
    W_0[4+(NY0) * 13] = 0;
    W_0[5+(NY0) * 0] = 0;
    W_0[5+(NY0) * 1] = 0;
    W_0[5+(NY0) * 2] = 0;
    W_0[5+(NY0) * 3] = 0;
    W_0[5+(NY0) * 4] = 0;
    W_0[5+(NY0) * 5] = 0.03;
    W_0[5+(NY0) * 6] = 0;
    W_0[5+(NY0) * 7] = 0;
    W_0[5+(NY0) * 8] = 0;
    W_0[5+(NY0) * 9] = 0;
    W_0[5+(NY0) * 10] = 0;
    W_0[5+(NY0) * 11] = 0;
    W_0[5+(NY0) * 12] = 0;
    W_0[5+(NY0) * 13] = 0;
    W_0[6+(NY0) * 0] = 0;
    W_0[6+(NY0) * 1] = 0;
    W_0[6+(NY0) * 2] = 0;
    W_0[6+(NY0) * 3] = 0;
    W_0[6+(NY0) * 4] = 0;
    W_0[6+(NY0) * 5] = 0;
    W_0[6+(NY0) * 6] = 0.03;
    W_0[6+(NY0) * 7] = 0;
    W_0[6+(NY0) * 8] = 0;
    W_0[6+(NY0) * 9] = 0;
    W_0[6+(NY0) * 10] = 0;
    W_0[6+(NY0) * 11] = 0;
    W_0[6+(NY0) * 12] = 0;
    W_0[6+(NY0) * 13] = 0;
    W_0[7+(NY0) * 0] = 0;
    W_0[7+(NY0) * 1] = 0;
    W_0[7+(NY0) * 2] = 0;
    W_0[7+(NY0) * 3] = 0;
    W_0[7+(NY0) * 4] = 0;
    W_0[7+(NY0) * 5] = 0;
    W_0[7+(NY0) * 6] = 0;
    W_0[7+(NY0) * 7] = 0.05;
    W_0[7+(NY0) * 8] = 0;
    W_0[7+(NY0) * 9] = 0;
    W_0[7+(NY0) * 10] = 0;
    W_0[7+(NY0) * 11] = 0;
    W_0[7+(NY0) * 12] = 0;
    W_0[7+(NY0) * 13] = 0;
    W_0[8+(NY0) * 0] = 0;
    W_0[8+(NY0) * 1] = 0;
    W_0[8+(NY0) * 2] = 0;
    W_0[8+(NY0) * 3] = 0;
    W_0[8+(NY0) * 4] = 0;
    W_0[8+(NY0) * 5] = 0;
    W_0[8+(NY0) * 6] = 0;
    W_0[8+(NY0) * 7] = 0;
    W_0[8+(NY0) * 8] = 0.05;
    W_0[8+(NY0) * 9] = 0;
    W_0[8+(NY0) * 10] = 0;
    W_0[8+(NY0) * 11] = 0;
    W_0[8+(NY0) * 12] = 0;
    W_0[8+(NY0) * 13] = 0;
    W_0[9+(NY0) * 0] = 0;
    W_0[9+(NY0) * 1] = 0;
    W_0[9+(NY0) * 2] = 0;
    W_0[9+(NY0) * 3] = 0;
    W_0[9+(NY0) * 4] = 0;
    W_0[9+(NY0) * 5] = 0;
    W_0[9+(NY0) * 6] = 0;
    W_0[9+(NY0) * 7] = 0;
    W_0[9+(NY0) * 8] = 0;
    W_0[9+(NY0) * 9] = 0.05;
    W_0[9+(NY0) * 10] = 0;
    W_0[9+(NY0) * 11] = 0;
    W_0[9+(NY0) * 12] = 0;
    W_0[9+(NY0) * 13] = 0;
    W_0[10+(NY0) * 0] = 0;
    W_0[10+(NY0) * 1] = 0;
    W_0[10+(NY0) * 2] = 0;
    W_0[10+(NY0) * 3] = 0;
    W_0[10+(NY0) * 4] = 0;
    W_0[10+(NY0) * 5] = 0;
    W_0[10+(NY0) * 6] = 0;
    W_0[10+(NY0) * 7] = 0;
    W_0[10+(NY0) * 8] = 0;
    W_0[10+(NY0) * 9] = 0;
    W_0[10+(NY0) * 10] = 2;
    W_0[10+(NY0) * 11] = 0;
    W_0[10+(NY0) * 12] = 0;
    W_0[10+(NY0) * 13] = 0;
    W_0[11+(NY0) * 0] = 0;
    W_0[11+(NY0) * 1] = 0;
    W_0[11+(NY0) * 2] = 0;
    W_0[11+(NY0) * 3] = 0;
    W_0[11+(NY0) * 4] = 0;
    W_0[11+(NY0) * 5] = 0;
    W_0[11+(NY0) * 6] = 0;
    W_0[11+(NY0) * 7] = 0;
    W_0[11+(NY0) * 8] = 0;
    W_0[11+(NY0) * 9] = 0;
    W_0[11+(NY0) * 10] = 0;
    W_0[11+(NY0) * 11] = 2;
    W_0[11+(NY0) * 12] = 0;
    W_0[11+(NY0) * 13] = 0;
    W_0[12+(NY0) * 0] = 0;
    W_0[12+(NY0) * 1] = 0;
    W_0[12+(NY0) * 2] = 0;
    W_0[12+(NY0) * 3] = 0;
    W_0[12+(NY0) * 4] = 0;
    W_0[12+(NY0) * 5] = 0;
    W_0[12+(NY0) * 6] = 0;
    W_0[12+(NY0) * 7] = 0;
    W_0[12+(NY0) * 8] = 0;
    W_0[12+(NY0) * 9] = 0;
    W_0[12+(NY0) * 10] = 0;
    W_0[12+(NY0) * 11] = 0;
    W_0[12+(NY0) * 12] = 5;
    W_0[12+(NY0) * 13] = 0;
    W_0[13+(NY0) * 0] = 0;
    W_0[13+(NY0) * 1] = 0;
    W_0[13+(NY0) * 2] = 0;
    W_0[13+(NY0) * 3] = 0;
    W_0[13+(NY0) * 4] = 0;
    W_0[13+(NY0) * 5] = 0;
    W_0[13+(NY0) * 6] = 0;
    W_0[13+(NY0) * 7] = 0;
    W_0[13+(NY0) * 8] = 0;
    W_0[13+(NY0) * 9] = 0;
    W_0[13+(NY0) * 10] = 0;
    W_0[13+(NY0) * 11] = 0;
    W_0[13+(NY0) * 12] = 0;
    W_0[13+(NY0) * 13] = 0.6;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "W", W_0);

    double yref_0[NY0];
    
    yref_0[0] = 0;
    yref_0[1] = 0;
    yref_0[2] = 0;
    yref_0[3] = 1;
    yref_0[4] = 0;
    yref_0[5] = 0;
    yref_0[6] = 0;
    yref_0[7] = 0;
    yref_0[8] = 0;
    yref_0[9] = 0;
    yref_0[10] = 0;
    yref_0[11] = 0;
    yref_0[12] = 0;
    yref_0[13] = 9.8066;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "yref", yref_0);



    double W[NY*NY];
    
    W[0+(NY) * 0] = 20;
    W[0+(NY) * 1] = 0;
    W[0+(NY) * 2] = 0;
    W[0+(NY) * 3] = 0;
    W[0+(NY) * 4] = 0;
    W[0+(NY) * 5] = 0;
    W[0+(NY) * 6] = 0;
    W[0+(NY) * 7] = 0;
    W[0+(NY) * 8] = 0;
    W[0+(NY) * 9] = 0;
    W[0+(NY) * 10] = 0;
    W[0+(NY) * 11] = 0;
    W[0+(NY) * 12] = 0;
    W[0+(NY) * 13] = 0;
    W[1+(NY) * 0] = 0;
    W[1+(NY) * 1] = 20;
    W[1+(NY) * 2] = 0;
    W[1+(NY) * 3] = 0;
    W[1+(NY) * 4] = 0;
    W[1+(NY) * 5] = 0;
    W[1+(NY) * 6] = 0;
    W[1+(NY) * 7] = 0;
    W[1+(NY) * 8] = 0;
    W[1+(NY) * 9] = 0;
    W[1+(NY) * 10] = 0;
    W[1+(NY) * 11] = 0;
    W[1+(NY) * 12] = 0;
    W[1+(NY) * 13] = 0;
    W[2+(NY) * 0] = 0;
    W[2+(NY) * 1] = 0;
    W[2+(NY) * 2] = 20;
    W[2+(NY) * 3] = 0;
    W[2+(NY) * 4] = 0;
    W[2+(NY) * 5] = 0;
    W[2+(NY) * 6] = 0;
    W[2+(NY) * 7] = 0;
    W[2+(NY) * 8] = 0;
    W[2+(NY) * 9] = 0;
    W[2+(NY) * 10] = 0;
    W[2+(NY) * 11] = 0;
    W[2+(NY) * 12] = 0;
    W[2+(NY) * 13] = 0;
    W[3+(NY) * 0] = 0;
    W[3+(NY) * 1] = 0;
    W[3+(NY) * 2] = 0;
    W[3+(NY) * 3] = 0.03;
    W[3+(NY) * 4] = 0;
    W[3+(NY) * 5] = 0;
    W[3+(NY) * 6] = 0;
    W[3+(NY) * 7] = 0;
    W[3+(NY) * 8] = 0;
    W[3+(NY) * 9] = 0;
    W[3+(NY) * 10] = 0;
    W[3+(NY) * 11] = 0;
    W[3+(NY) * 12] = 0;
    W[3+(NY) * 13] = 0;
    W[4+(NY) * 0] = 0;
    W[4+(NY) * 1] = 0;
    W[4+(NY) * 2] = 0;
    W[4+(NY) * 3] = 0;
    W[4+(NY) * 4] = 0.03;
    W[4+(NY) * 5] = 0;
    W[4+(NY) * 6] = 0;
    W[4+(NY) * 7] = 0;
    W[4+(NY) * 8] = 0;
    W[4+(NY) * 9] = 0;
    W[4+(NY) * 10] = 0;
    W[4+(NY) * 11] = 0;
    W[4+(NY) * 12] = 0;
    W[4+(NY) * 13] = 0;
    W[5+(NY) * 0] = 0;
    W[5+(NY) * 1] = 0;
    W[5+(NY) * 2] = 0;
    W[5+(NY) * 3] = 0;
    W[5+(NY) * 4] = 0;
    W[5+(NY) * 5] = 0.03;
    W[5+(NY) * 6] = 0;
    W[5+(NY) * 7] = 0;
    W[5+(NY) * 8] = 0;
    W[5+(NY) * 9] = 0;
    W[5+(NY) * 10] = 0;
    W[5+(NY) * 11] = 0;
    W[5+(NY) * 12] = 0;
    W[5+(NY) * 13] = 0;
    W[6+(NY) * 0] = 0;
    W[6+(NY) * 1] = 0;
    W[6+(NY) * 2] = 0;
    W[6+(NY) * 3] = 0;
    W[6+(NY) * 4] = 0;
    W[6+(NY) * 5] = 0;
    W[6+(NY) * 6] = 0.03;
    W[6+(NY) * 7] = 0;
    W[6+(NY) * 8] = 0;
    W[6+(NY) * 9] = 0;
    W[6+(NY) * 10] = 0;
    W[6+(NY) * 11] = 0;
    W[6+(NY) * 12] = 0;
    W[6+(NY) * 13] = 0;
    W[7+(NY) * 0] = 0;
    W[7+(NY) * 1] = 0;
    W[7+(NY) * 2] = 0;
    W[7+(NY) * 3] = 0;
    W[7+(NY) * 4] = 0;
    W[7+(NY) * 5] = 0;
    W[7+(NY) * 6] = 0;
    W[7+(NY) * 7] = 0.05;
    W[7+(NY) * 8] = 0;
    W[7+(NY) * 9] = 0;
    W[7+(NY) * 10] = 0;
    W[7+(NY) * 11] = 0;
    W[7+(NY) * 12] = 0;
    W[7+(NY) * 13] = 0;
    W[8+(NY) * 0] = 0;
    W[8+(NY) * 1] = 0;
    W[8+(NY) * 2] = 0;
    W[8+(NY) * 3] = 0;
    W[8+(NY) * 4] = 0;
    W[8+(NY) * 5] = 0;
    W[8+(NY) * 6] = 0;
    W[8+(NY) * 7] = 0;
    W[8+(NY) * 8] = 0.05;
    W[8+(NY) * 9] = 0;
    W[8+(NY) * 10] = 0;
    W[8+(NY) * 11] = 0;
    W[8+(NY) * 12] = 0;
    W[8+(NY) * 13] = 0;
    W[9+(NY) * 0] = 0;
    W[9+(NY) * 1] = 0;
    W[9+(NY) * 2] = 0;
    W[9+(NY) * 3] = 0;
    W[9+(NY) * 4] = 0;
    W[9+(NY) * 5] = 0;
    W[9+(NY) * 6] = 0;
    W[9+(NY) * 7] = 0;
    W[9+(NY) * 8] = 0;
    W[9+(NY) * 9] = 0.05;
    W[9+(NY) * 10] = 0;
    W[9+(NY) * 11] = 0;
    W[9+(NY) * 12] = 0;
    W[9+(NY) * 13] = 0;
    W[10+(NY) * 0] = 0;
    W[10+(NY) * 1] = 0;
    W[10+(NY) * 2] = 0;
    W[10+(NY) * 3] = 0;
    W[10+(NY) * 4] = 0;
    W[10+(NY) * 5] = 0;
    W[10+(NY) * 6] = 0;
    W[10+(NY) * 7] = 0;
    W[10+(NY) * 8] = 0;
    W[10+(NY) * 9] = 0;
    W[10+(NY) * 10] = 2;
    W[10+(NY) * 11] = 0;
    W[10+(NY) * 12] = 0;
    W[10+(NY) * 13] = 0;
    W[11+(NY) * 0] = 0;
    W[11+(NY) * 1] = 0;
    W[11+(NY) * 2] = 0;
    W[11+(NY) * 3] = 0;
    W[11+(NY) * 4] = 0;
    W[11+(NY) * 5] = 0;
    W[11+(NY) * 6] = 0;
    W[11+(NY) * 7] = 0;
    W[11+(NY) * 8] = 0;
    W[11+(NY) * 9] = 0;
    W[11+(NY) * 10] = 0;
    W[11+(NY) * 11] = 2;
    W[11+(NY) * 12] = 0;
    W[11+(NY) * 13] = 0;
    W[12+(NY) * 0] = 0;
    W[12+(NY) * 1] = 0;
    W[12+(NY) * 2] = 0;
    W[12+(NY) * 3] = 0;
    W[12+(NY) * 4] = 0;
    W[12+(NY) * 5] = 0;
    W[12+(NY) * 6] = 0;
    W[12+(NY) * 7] = 0;
    W[12+(NY) * 8] = 0;
    W[12+(NY) * 9] = 0;
    W[12+(NY) * 10] = 0;
    W[12+(NY) * 11] = 0;
    W[12+(NY) * 12] = 5;
    W[12+(NY) * 13] = 0;
    W[13+(NY) * 0] = 0;
    W[13+(NY) * 1] = 0;
    W[13+(NY) * 2] = 0;
    W[13+(NY) * 3] = 0;
    W[13+(NY) * 4] = 0;
    W[13+(NY) * 5] = 0;
    W[13+(NY) * 6] = 0;
    W[13+(NY) * 7] = 0;
    W[13+(NY) * 8] = 0;
    W[13+(NY) * 9] = 0;
    W[13+(NY) * 10] = 0;
    W[13+(NY) * 11] = 0;
    W[13+(NY) * 12] = 0;
    W[13+(NY) * 13] = 0.6;

    double yref[NY];
    
    yref[0] = 0;
    yref[1] = 0;
    yref[2] = 0;
    yref[3] = 1;
    yref[4] = 0;
    yref[5] = 0;
    yref[6] = 0;
    yref[7] = 0;
    yref[8] = 0;
    yref[9] = 0;
    yref[10] = 0;
    yref[11] = 0;
    yref[12] = 0;
    yref[13] = 9.8066;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
    }


    double Vx_0[NY0*NX];
    
    Vx_0[0+(NY0) * 0] = 1;
    Vx_0[0+(NY0) * 1] = 0;
    Vx_0[0+(NY0) * 2] = 0;
    Vx_0[0+(NY0) * 3] = 0;
    Vx_0[0+(NY0) * 4] = 0;
    Vx_0[0+(NY0) * 5] = 0;
    Vx_0[0+(NY0) * 6] = 0;
    Vx_0[0+(NY0) * 7] = 0;
    Vx_0[0+(NY0) * 8] = 0;
    Vx_0[0+(NY0) * 9] = 0;
    Vx_0[1+(NY0) * 0] = 0;
    Vx_0[1+(NY0) * 1] = 1;
    Vx_0[1+(NY0) * 2] = 0;
    Vx_0[1+(NY0) * 3] = 0;
    Vx_0[1+(NY0) * 4] = 0;
    Vx_0[1+(NY0) * 5] = 0;
    Vx_0[1+(NY0) * 6] = 0;
    Vx_0[1+(NY0) * 7] = 0;
    Vx_0[1+(NY0) * 8] = 0;
    Vx_0[1+(NY0) * 9] = 0;
    Vx_0[2+(NY0) * 0] = 0;
    Vx_0[2+(NY0) * 1] = 0;
    Vx_0[2+(NY0) * 2] = 1;
    Vx_0[2+(NY0) * 3] = 0;
    Vx_0[2+(NY0) * 4] = 0;
    Vx_0[2+(NY0) * 5] = 0;
    Vx_0[2+(NY0) * 6] = 0;
    Vx_0[2+(NY0) * 7] = 0;
    Vx_0[2+(NY0) * 8] = 0;
    Vx_0[2+(NY0) * 9] = 0;
    Vx_0[3+(NY0) * 0] = 0;
    Vx_0[3+(NY0) * 1] = 0;
    Vx_0[3+(NY0) * 2] = 0;
    Vx_0[3+(NY0) * 3] = 1;
    Vx_0[3+(NY0) * 4] = 0;
    Vx_0[3+(NY0) * 5] = 0;
    Vx_0[3+(NY0) * 6] = 0;
    Vx_0[3+(NY0) * 7] = 0;
    Vx_0[3+(NY0) * 8] = 0;
    Vx_0[3+(NY0) * 9] = 0;
    Vx_0[4+(NY0) * 0] = 0;
    Vx_0[4+(NY0) * 1] = 0;
    Vx_0[4+(NY0) * 2] = 0;
    Vx_0[4+(NY0) * 3] = 0;
    Vx_0[4+(NY0) * 4] = 1;
    Vx_0[4+(NY0) * 5] = 0;
    Vx_0[4+(NY0) * 6] = 0;
    Vx_0[4+(NY0) * 7] = 0;
    Vx_0[4+(NY0) * 8] = 0;
    Vx_0[4+(NY0) * 9] = 0;
    Vx_0[5+(NY0) * 0] = 0;
    Vx_0[5+(NY0) * 1] = 0;
    Vx_0[5+(NY0) * 2] = 0;
    Vx_0[5+(NY0) * 3] = 0;
    Vx_0[5+(NY0) * 4] = 0;
    Vx_0[5+(NY0) * 5] = 1;
    Vx_0[5+(NY0) * 6] = 0;
    Vx_0[5+(NY0) * 7] = 0;
    Vx_0[5+(NY0) * 8] = 0;
    Vx_0[5+(NY0) * 9] = 0;
    Vx_0[6+(NY0) * 0] = 0;
    Vx_0[6+(NY0) * 1] = 0;
    Vx_0[6+(NY0) * 2] = 0;
    Vx_0[6+(NY0) * 3] = 0;
    Vx_0[6+(NY0) * 4] = 0;
    Vx_0[6+(NY0) * 5] = 0;
    Vx_0[6+(NY0) * 6] = 1;
    Vx_0[6+(NY0) * 7] = 0;
    Vx_0[6+(NY0) * 8] = 0;
    Vx_0[6+(NY0) * 9] = 0;
    Vx_0[7+(NY0) * 0] = 0;
    Vx_0[7+(NY0) * 1] = 0;
    Vx_0[7+(NY0) * 2] = 0;
    Vx_0[7+(NY0) * 3] = 0;
    Vx_0[7+(NY0) * 4] = 0;
    Vx_0[7+(NY0) * 5] = 0;
    Vx_0[7+(NY0) * 6] = 0;
    Vx_0[7+(NY0) * 7] = 1;
    Vx_0[7+(NY0) * 8] = 0;
    Vx_0[7+(NY0) * 9] = 0;
    Vx_0[8+(NY0) * 0] = 0;
    Vx_0[8+(NY0) * 1] = 0;
    Vx_0[8+(NY0) * 2] = 0;
    Vx_0[8+(NY0) * 3] = 0;
    Vx_0[8+(NY0) * 4] = 0;
    Vx_0[8+(NY0) * 5] = 0;
    Vx_0[8+(NY0) * 6] = 0;
    Vx_0[8+(NY0) * 7] = 0;
    Vx_0[8+(NY0) * 8] = 1;
    Vx_0[8+(NY0) * 9] = 0;
    Vx_0[9+(NY0) * 0] = 0;
    Vx_0[9+(NY0) * 1] = 0;
    Vx_0[9+(NY0) * 2] = 0;
    Vx_0[9+(NY0) * 3] = 0;
    Vx_0[9+(NY0) * 4] = 0;
    Vx_0[9+(NY0) * 5] = 0;
    Vx_0[9+(NY0) * 6] = 0;
    Vx_0[9+(NY0) * 7] = 0;
    Vx_0[9+(NY0) * 8] = 0;
    Vx_0[9+(NY0) * 9] = 1;
    Vx_0[10+(NY0) * 0] = 0;
    Vx_0[10+(NY0) * 1] = 0;
    Vx_0[10+(NY0) * 2] = 0;
    Vx_0[10+(NY0) * 3] = 0;
    Vx_0[10+(NY0) * 4] = 0;
    Vx_0[10+(NY0) * 5] = 0;
    Vx_0[10+(NY0) * 6] = 0;
    Vx_0[10+(NY0) * 7] = 0;
    Vx_0[10+(NY0) * 8] = 0;
    Vx_0[10+(NY0) * 9] = 0;
    Vx_0[11+(NY0) * 0] = 0;
    Vx_0[11+(NY0) * 1] = 0;
    Vx_0[11+(NY0) * 2] = 0;
    Vx_0[11+(NY0) * 3] = 0;
    Vx_0[11+(NY0) * 4] = 0;
    Vx_0[11+(NY0) * 5] = 0;
    Vx_0[11+(NY0) * 6] = 0;
    Vx_0[11+(NY0) * 7] = 0;
    Vx_0[11+(NY0) * 8] = 0;
    Vx_0[11+(NY0) * 9] = 0;
    Vx_0[12+(NY0) * 0] = 0;
    Vx_0[12+(NY0) * 1] = 0;
    Vx_0[12+(NY0) * 2] = 0;
    Vx_0[12+(NY0) * 3] = 0;
    Vx_0[12+(NY0) * 4] = 0;
    Vx_0[12+(NY0) * 5] = 0;
    Vx_0[12+(NY0) * 6] = 0;
    Vx_0[12+(NY0) * 7] = 0;
    Vx_0[12+(NY0) * 8] = 0;
    Vx_0[12+(NY0) * 9] = 0;
    Vx_0[13+(NY0) * 0] = 0;
    Vx_0[13+(NY0) * 1] = 0;
    Vx_0[13+(NY0) * 2] = 0;
    Vx_0[13+(NY0) * 3] = 0;
    Vx_0[13+(NY0) * 4] = 0;
    Vx_0[13+(NY0) * 5] = 0;
    Vx_0[13+(NY0) * 6] = 0;
    Vx_0[13+(NY0) * 7] = 0;
    Vx_0[13+(NY0) * 8] = 0;
    Vx_0[13+(NY0) * 9] = 0;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "Vx", Vx_0);


    double Vu_0[NY0*NU];
    
    Vu_0[0+(NY0) * 0] = 0;
    Vu_0[0+(NY0) * 1] = 0;
    Vu_0[0+(NY0) * 2] = 0;
    Vu_0[0+(NY0) * 3] = 0;
    Vu_0[1+(NY0) * 0] = 0;
    Vu_0[1+(NY0) * 1] = 0;
    Vu_0[1+(NY0) * 2] = 0;
    Vu_0[1+(NY0) * 3] = 0;
    Vu_0[2+(NY0) * 0] = 0;
    Vu_0[2+(NY0) * 1] = 0;
    Vu_0[2+(NY0) * 2] = 0;
    Vu_0[2+(NY0) * 3] = 0;
    Vu_0[3+(NY0) * 0] = 0;
    Vu_0[3+(NY0) * 1] = 0;
    Vu_0[3+(NY0) * 2] = 0;
    Vu_0[3+(NY0) * 3] = 0;
    Vu_0[4+(NY0) * 0] = 0;
    Vu_0[4+(NY0) * 1] = 0;
    Vu_0[4+(NY0) * 2] = 0;
    Vu_0[4+(NY0) * 3] = 0;
    Vu_0[5+(NY0) * 0] = 0;
    Vu_0[5+(NY0) * 1] = 0;
    Vu_0[5+(NY0) * 2] = 0;
    Vu_0[5+(NY0) * 3] = 0;
    Vu_0[6+(NY0) * 0] = 0;
    Vu_0[6+(NY0) * 1] = 0;
    Vu_0[6+(NY0) * 2] = 0;
    Vu_0[6+(NY0) * 3] = 0;
    Vu_0[7+(NY0) * 0] = 0;
    Vu_0[7+(NY0) * 1] = 0;
    Vu_0[7+(NY0) * 2] = 0;
    Vu_0[7+(NY0) * 3] = 0;
    Vu_0[8+(NY0) * 0] = 0;
    Vu_0[8+(NY0) * 1] = 0;
    Vu_0[8+(NY0) * 2] = 0;
    Vu_0[8+(NY0) * 3] = 0;
    Vu_0[9+(NY0) * 0] = 0;
    Vu_0[9+(NY0) * 1] = 0;
    Vu_0[9+(NY0) * 2] = 0;
    Vu_0[9+(NY0) * 3] = 0;
    Vu_0[10+(NY0) * 0] = 1;
    Vu_0[10+(NY0) * 1] = 0;
    Vu_0[10+(NY0) * 2] = 0;
    Vu_0[10+(NY0) * 3] = 0;
    Vu_0[11+(NY0) * 0] = 0;
    Vu_0[11+(NY0) * 1] = 1;
    Vu_0[11+(NY0) * 2] = 0;
    Vu_0[11+(NY0) * 3] = 0;
    Vu_0[12+(NY0) * 0] = 0;
    Vu_0[12+(NY0) * 1] = 0;
    Vu_0[12+(NY0) * 2] = 1;
    Vu_0[12+(NY0) * 3] = 0;
    Vu_0[13+(NY0) * 0] = 0;
    Vu_0[13+(NY0) * 1] = 0;
    Vu_0[13+(NY0) * 2] = 0;
    Vu_0[13+(NY0) * 3] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "Vu", Vu_0);


    double Vx[NY*NX];
    
    Vx[0+(NY) * 0] = 1;
    Vx[0+(NY) * 1] = 0;
    Vx[0+(NY) * 2] = 0;
    Vx[0+(NY) * 3] = 0;
    Vx[0+(NY) * 4] = 0;
    Vx[0+(NY) * 5] = 0;
    Vx[0+(NY) * 6] = 0;
    Vx[0+(NY) * 7] = 0;
    Vx[0+(NY) * 8] = 0;
    Vx[0+(NY) * 9] = 0;
    Vx[1+(NY) * 0] = 0;
    Vx[1+(NY) * 1] = 1;
    Vx[1+(NY) * 2] = 0;
    Vx[1+(NY) * 3] = 0;
    Vx[1+(NY) * 4] = 0;
    Vx[1+(NY) * 5] = 0;
    Vx[1+(NY) * 6] = 0;
    Vx[1+(NY) * 7] = 0;
    Vx[1+(NY) * 8] = 0;
    Vx[1+(NY) * 9] = 0;
    Vx[2+(NY) * 0] = 0;
    Vx[2+(NY) * 1] = 0;
    Vx[2+(NY) * 2] = 1;
    Vx[2+(NY) * 3] = 0;
    Vx[2+(NY) * 4] = 0;
    Vx[2+(NY) * 5] = 0;
    Vx[2+(NY) * 6] = 0;
    Vx[2+(NY) * 7] = 0;
    Vx[2+(NY) * 8] = 0;
    Vx[2+(NY) * 9] = 0;
    Vx[3+(NY) * 0] = 0;
    Vx[3+(NY) * 1] = 0;
    Vx[3+(NY) * 2] = 0;
    Vx[3+(NY) * 3] = 1;
    Vx[3+(NY) * 4] = 0;
    Vx[3+(NY) * 5] = 0;
    Vx[3+(NY) * 6] = 0;
    Vx[3+(NY) * 7] = 0;
    Vx[3+(NY) * 8] = 0;
    Vx[3+(NY) * 9] = 0;
    Vx[4+(NY) * 0] = 0;
    Vx[4+(NY) * 1] = 0;
    Vx[4+(NY) * 2] = 0;
    Vx[4+(NY) * 3] = 0;
    Vx[4+(NY) * 4] = 1;
    Vx[4+(NY) * 5] = 0;
    Vx[4+(NY) * 6] = 0;
    Vx[4+(NY) * 7] = 0;
    Vx[4+(NY) * 8] = 0;
    Vx[4+(NY) * 9] = 0;
    Vx[5+(NY) * 0] = 0;
    Vx[5+(NY) * 1] = 0;
    Vx[5+(NY) * 2] = 0;
    Vx[5+(NY) * 3] = 0;
    Vx[5+(NY) * 4] = 0;
    Vx[5+(NY) * 5] = 1;
    Vx[5+(NY) * 6] = 0;
    Vx[5+(NY) * 7] = 0;
    Vx[5+(NY) * 8] = 0;
    Vx[5+(NY) * 9] = 0;
    Vx[6+(NY) * 0] = 0;
    Vx[6+(NY) * 1] = 0;
    Vx[6+(NY) * 2] = 0;
    Vx[6+(NY) * 3] = 0;
    Vx[6+(NY) * 4] = 0;
    Vx[6+(NY) * 5] = 0;
    Vx[6+(NY) * 6] = 1;
    Vx[6+(NY) * 7] = 0;
    Vx[6+(NY) * 8] = 0;
    Vx[6+(NY) * 9] = 0;
    Vx[7+(NY) * 0] = 0;
    Vx[7+(NY) * 1] = 0;
    Vx[7+(NY) * 2] = 0;
    Vx[7+(NY) * 3] = 0;
    Vx[7+(NY) * 4] = 0;
    Vx[7+(NY) * 5] = 0;
    Vx[7+(NY) * 6] = 0;
    Vx[7+(NY) * 7] = 1;
    Vx[7+(NY) * 8] = 0;
    Vx[7+(NY) * 9] = 0;
    Vx[8+(NY) * 0] = 0;
    Vx[8+(NY) * 1] = 0;
    Vx[8+(NY) * 2] = 0;
    Vx[8+(NY) * 3] = 0;
    Vx[8+(NY) * 4] = 0;
    Vx[8+(NY) * 5] = 0;
    Vx[8+(NY) * 6] = 0;
    Vx[8+(NY) * 7] = 0;
    Vx[8+(NY) * 8] = 1;
    Vx[8+(NY) * 9] = 0;
    Vx[9+(NY) * 0] = 0;
    Vx[9+(NY) * 1] = 0;
    Vx[9+(NY) * 2] = 0;
    Vx[9+(NY) * 3] = 0;
    Vx[9+(NY) * 4] = 0;
    Vx[9+(NY) * 5] = 0;
    Vx[9+(NY) * 6] = 0;
    Vx[9+(NY) * 7] = 0;
    Vx[9+(NY) * 8] = 0;
    Vx[9+(NY) * 9] = 1;
    Vx[10+(NY) * 0] = 0;
    Vx[10+(NY) * 1] = 0;
    Vx[10+(NY) * 2] = 0;
    Vx[10+(NY) * 3] = 0;
    Vx[10+(NY) * 4] = 0;
    Vx[10+(NY) * 5] = 0;
    Vx[10+(NY) * 6] = 0;
    Vx[10+(NY) * 7] = 0;
    Vx[10+(NY) * 8] = 0;
    Vx[10+(NY) * 9] = 0;
    Vx[11+(NY) * 0] = 0;
    Vx[11+(NY) * 1] = 0;
    Vx[11+(NY) * 2] = 0;
    Vx[11+(NY) * 3] = 0;
    Vx[11+(NY) * 4] = 0;
    Vx[11+(NY) * 5] = 0;
    Vx[11+(NY) * 6] = 0;
    Vx[11+(NY) * 7] = 0;
    Vx[11+(NY) * 8] = 0;
    Vx[11+(NY) * 9] = 0;
    Vx[12+(NY) * 0] = 0;
    Vx[12+(NY) * 1] = 0;
    Vx[12+(NY) * 2] = 0;
    Vx[12+(NY) * 3] = 0;
    Vx[12+(NY) * 4] = 0;
    Vx[12+(NY) * 5] = 0;
    Vx[12+(NY) * 6] = 0;
    Vx[12+(NY) * 7] = 0;
    Vx[12+(NY) * 8] = 0;
    Vx[12+(NY) * 9] = 0;
    Vx[13+(NY) * 0] = 0;
    Vx[13+(NY) * 1] = 0;
    Vx[13+(NY) * 2] = 0;
    Vx[13+(NY) * 3] = 0;
    Vx[13+(NY) * 4] = 0;
    Vx[13+(NY) * 5] = 0;
    Vx[13+(NY) * 6] = 0;
    Vx[13+(NY) * 7] = 0;
    Vx[13+(NY) * 8] = 0;
    Vx[13+(NY) * 9] = 0;
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Vx", Vx);
    }


    double Vu[NY*NU];
    
    Vu[0+(NY) * 0] = 0;
    Vu[0+(NY) * 1] = 0;
    Vu[0+(NY) * 2] = 0;
    Vu[0+(NY) * 3] = 0;
    Vu[1+(NY) * 0] = 0;
    Vu[1+(NY) * 1] = 0;
    Vu[1+(NY) * 2] = 0;
    Vu[1+(NY) * 3] = 0;
    Vu[2+(NY) * 0] = 0;
    Vu[2+(NY) * 1] = 0;
    Vu[2+(NY) * 2] = 0;
    Vu[2+(NY) * 3] = 0;
    Vu[3+(NY) * 0] = 0;
    Vu[3+(NY) * 1] = 0;
    Vu[3+(NY) * 2] = 0;
    Vu[3+(NY) * 3] = 0;
    Vu[4+(NY) * 0] = 0;
    Vu[4+(NY) * 1] = 0;
    Vu[4+(NY) * 2] = 0;
    Vu[4+(NY) * 3] = 0;
    Vu[5+(NY) * 0] = 0;
    Vu[5+(NY) * 1] = 0;
    Vu[5+(NY) * 2] = 0;
    Vu[5+(NY) * 3] = 0;
    Vu[6+(NY) * 0] = 0;
    Vu[6+(NY) * 1] = 0;
    Vu[6+(NY) * 2] = 0;
    Vu[6+(NY) * 3] = 0;
    Vu[7+(NY) * 0] = 0;
    Vu[7+(NY) * 1] = 0;
    Vu[7+(NY) * 2] = 0;
    Vu[7+(NY) * 3] = 0;
    Vu[8+(NY) * 0] = 0;
    Vu[8+(NY) * 1] = 0;
    Vu[8+(NY) * 2] = 0;
    Vu[8+(NY) * 3] = 0;
    Vu[9+(NY) * 0] = 0;
    Vu[9+(NY) * 1] = 0;
    Vu[9+(NY) * 2] = 0;
    Vu[9+(NY) * 3] = 0;
    Vu[10+(NY) * 0] = 1;
    Vu[10+(NY) * 1] = 0;
    Vu[10+(NY) * 2] = 0;
    Vu[10+(NY) * 3] = 0;
    Vu[11+(NY) * 0] = 0;
    Vu[11+(NY) * 1] = 1;
    Vu[11+(NY) * 2] = 0;
    Vu[11+(NY) * 3] = 0;
    Vu[12+(NY) * 0] = 0;
    Vu[12+(NY) * 1] = 0;
    Vu[12+(NY) * 2] = 1;
    Vu[12+(NY) * 3] = 0;
    Vu[13+(NY) * 0] = 0;
    Vu[13+(NY) * 1] = 0;
    Vu[13+(NY) * 2] = 0;
    Vu[13+(NY) * 3] = 1;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Vu", Vu);
    }







    // terminal cost


    double yref_e[NYN];
    
    yref_e[0] = 0;
    yref_e[1] = 0;
    yref_e[2] = 0;
    yref_e[3] = 0;
    yref_e[4] = 0;
    yref_e[5] = 0;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", yref_e);

    double W_e[NYN*NYN];
    
    W_e[0+(NYN) * 0] = 10;
    W_e[0+(NYN) * 1] = 0;
    W_e[0+(NYN) * 2] = 0;
    W_e[0+(NYN) * 3] = 0;
    W_e[0+(NYN) * 4] = 0;
    W_e[0+(NYN) * 5] = 0;
    W_e[1+(NYN) * 0] = 0;
    W_e[1+(NYN) * 1] = 10;
    W_e[1+(NYN) * 2] = 0;
    W_e[1+(NYN) * 3] = 0;
    W_e[1+(NYN) * 4] = 0;
    W_e[1+(NYN) * 5] = 0;
    W_e[2+(NYN) * 0] = 0;
    W_e[2+(NYN) * 1] = 0;
    W_e[2+(NYN) * 2] = 20;
    W_e[2+(NYN) * 3] = 0;
    W_e[2+(NYN) * 4] = 0;
    W_e[2+(NYN) * 5] = 0;
    W_e[3+(NYN) * 0] = 0;
    W_e[3+(NYN) * 1] = 0;
    W_e[3+(NYN) * 2] = 0;
    W_e[3+(NYN) * 3] = 0.05;
    W_e[3+(NYN) * 4] = 0;
    W_e[3+(NYN) * 5] = 0;
    W_e[4+(NYN) * 0] = 0;
    W_e[4+(NYN) * 1] = 0;
    W_e[4+(NYN) * 2] = 0;
    W_e[4+(NYN) * 3] = 0;
    W_e[4+(NYN) * 4] = 0.05;
    W_e[4+(NYN) * 5] = 0;
    W_e[5+(NYN) * 0] = 0;
    W_e[5+(NYN) * 1] = 0;
    W_e[5+(NYN) * 2] = 0;
    W_e[5+(NYN) * 3] = 0;
    W_e[5+(NYN) * 4] = 0;
    W_e[5+(NYN) * 5] = 0.05;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", W_e);
    double Vx_e[NYN*NX];
    
    Vx_e[0+(NYN) * 0] = 1;
    Vx_e[0+(NYN) * 1] = 0;
    Vx_e[0+(NYN) * 2] = 0;
    Vx_e[0+(NYN) * 3] = 0;
    Vx_e[0+(NYN) * 4] = 0;
    Vx_e[0+(NYN) * 5] = 0;
    Vx_e[0+(NYN) * 6] = 0;
    Vx_e[0+(NYN) * 7] = 0;
    Vx_e[0+(NYN) * 8] = 0;
    Vx_e[0+(NYN) * 9] = 0;
    Vx_e[1+(NYN) * 0] = 0;
    Vx_e[1+(NYN) * 1] = 1;
    Vx_e[1+(NYN) * 2] = 0;
    Vx_e[1+(NYN) * 3] = 0;
    Vx_e[1+(NYN) * 4] = 0;
    Vx_e[1+(NYN) * 5] = 0;
    Vx_e[1+(NYN) * 6] = 0;
    Vx_e[1+(NYN) * 7] = 0;
    Vx_e[1+(NYN) * 8] = 0;
    Vx_e[1+(NYN) * 9] = 0;
    Vx_e[2+(NYN) * 0] = 0;
    Vx_e[2+(NYN) * 1] = 0;
    Vx_e[2+(NYN) * 2] = 1;
    Vx_e[2+(NYN) * 3] = 0;
    Vx_e[2+(NYN) * 4] = 0;
    Vx_e[2+(NYN) * 5] = 0;
    Vx_e[2+(NYN) * 6] = 0;
    Vx_e[2+(NYN) * 7] = 0;
    Vx_e[2+(NYN) * 8] = 0;
    Vx_e[2+(NYN) * 9] = 0;
    Vx_e[3+(NYN) * 0] = 0;
    Vx_e[3+(NYN) * 1] = 0;
    Vx_e[3+(NYN) * 2] = 0;
    Vx_e[3+(NYN) * 3] = 0;
    Vx_e[3+(NYN) * 4] = 0;
    Vx_e[3+(NYN) * 5] = 0;
    Vx_e[3+(NYN) * 6] = 0;
    Vx_e[3+(NYN) * 7] = 1;
    Vx_e[3+(NYN) * 8] = 0;
    Vx_e[3+(NYN) * 9] = 0;
    Vx_e[4+(NYN) * 0] = 0;
    Vx_e[4+(NYN) * 1] = 0;
    Vx_e[4+(NYN) * 2] = 0;
    Vx_e[4+(NYN) * 3] = 0;
    Vx_e[4+(NYN) * 4] = 0;
    Vx_e[4+(NYN) * 5] = 0;
    Vx_e[4+(NYN) * 6] = 0;
    Vx_e[4+(NYN) * 7] = 0;
    Vx_e[4+(NYN) * 8] = 1;
    Vx_e[4+(NYN) * 9] = 0;
    Vx_e[5+(NYN) * 0] = 0;
    Vx_e[5+(NYN) * 1] = 0;
    Vx_e[5+(NYN) * 2] = 0;
    Vx_e[5+(NYN) * 3] = 0;
    Vx_e[5+(NYN) * 4] = 0;
    Vx_e[5+(NYN) * 5] = 0;
    Vx_e[5+(NYN) * 6] = 0;
    Vx_e[5+(NYN) * 7] = 0;
    Vx_e[5+(NYN) * 8] = 0;
    Vx_e[5+(NYN) * 9] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "Vx", Vx_e);



    /**** Constraints ****/

    // bounds for initial stage

    // x0
    int idxbx0[10];
    
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;
    idxbx0[5] = 5;
    idxbx0[6] = 6;
    idxbx0[7] = 7;
    idxbx0[8] = 8;
    idxbx0[9] = 9;

    double lbx0[10];
    double ubx0[10];
    
    lbx0[0] = 0;
    ubx0[0] = 0;
    lbx0[1] = 0;
    ubx0[1] = 0;
    lbx0[2] = 0;
    ubx0[2] = 0;
    lbx0[3] = 1;
    ubx0[3] = 1;
    lbx0[4] = 0;
    ubx0[4] = 0;
    lbx0[5] = 0;
    ubx0[5] = 0;
    lbx0[6] = 0;
    ubx0[6] = 0;
    lbx0[7] = 0;
    ubx0[7] = 0;
    lbx0[8] = 0;
    ubx0[8] = 0;
    lbx0[9] = 0;
    ubx0[9] = 0;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);


    // idxbxe_0
    int idxbxe_0[10];
    
    idxbxe_0[0] = 0;
    idxbxe_0[1] = 1;
    idxbxe_0[2] = 2;
    idxbxe_0[3] = 3;
    idxbxe_0[4] = 4;
    idxbxe_0[5] = 5;
    idxbxe_0[6] = 6;
    idxbxe_0[7] = 7;
    idxbxe_0[8] = 8;
    idxbxe_0[9] = 9;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbxe", idxbxe_0);


    /* constraints that are the same for initial and intermediate */



    // u
    int idxbu[NBU];
    
    idxbu[0] = 0;
    idxbu[1] = 1;
    idxbu[2] = 2;
    idxbu[3] = 3;
    double lbu[NBU];
    double ubu[NBU];
    
    lbu[0] = -6;
    ubu[0] = 6;
    lbu[1] = -6;
    ubu[1] = 6;
    lbu[2] = -3.14;
    ubu[2] = 3.14;
    lbu[3] = 2;
    ubu[3] = 14.7099;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu);
    }


















    /* terminal constraints */

















    /************************************************
    *  opts
    ************************************************/

    capsule->nlp_opts = ocp_nlp_solver_opts_create(nlp_config, nlp_dims);


    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "globalization", "fixed_step");

    int sim_method_num_steps[N];
    sim_method_num_steps[0] = 1;
    sim_method_num_steps[1] = 1;
    sim_method_num_steps[2] = 1;
    sim_method_num_steps[3] = 1;
    sim_method_num_steps[4] = 1;
    sim_method_num_steps[5] = 1;
    sim_method_num_steps[6] = 1;
    sim_method_num_steps[7] = 1;
    sim_method_num_steps[8] = 1;
    sim_method_num_steps[9] = 1;
    sim_method_num_steps[10] = 1;
    sim_method_num_steps[11] = 1;
    sim_method_num_steps[12] = 1;
    sim_method_num_steps[13] = 1;
    sim_method_num_steps[14] = 1;
    sim_method_num_steps[15] = 1;
    sim_method_num_steps[16] = 1;
    sim_method_num_steps[17] = 1;
    sim_method_num_steps[18] = 1;
    sim_method_num_steps[19] = 1;

    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, capsule->nlp_opts, i, "dynamics_num_steps", &sim_method_num_steps[i]);

    int sim_method_num_stages[N];
    sim_method_num_stages[0] = 4;
    sim_method_num_stages[1] = 4;
    sim_method_num_stages[2] = 4;
    sim_method_num_stages[3] = 4;
    sim_method_num_stages[4] = 4;
    sim_method_num_stages[5] = 4;
    sim_method_num_stages[6] = 4;
    sim_method_num_stages[7] = 4;
    sim_method_num_stages[8] = 4;
    sim_method_num_stages[9] = 4;
    sim_method_num_stages[10] = 4;
    sim_method_num_stages[11] = 4;
    sim_method_num_stages[12] = 4;
    sim_method_num_stages[13] = 4;
    sim_method_num_stages[14] = 4;
    sim_method_num_stages[15] = 4;
    sim_method_num_stages[16] = 4;
    sim_method_num_stages[17] = 4;
    sim_method_num_stages[18] = 4;
    sim_method_num_stages[19] = 4;

    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, capsule->nlp_opts, i, "dynamics_num_stages", &sim_method_num_stages[i]);

    int newton_iter_val = 3;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, capsule->nlp_opts, i, "dynamics_newton_iter", &newton_iter_val);

    bool tmp_bool = false;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, capsule->nlp_opts, i, "dynamics_jac_reuse", &tmp_bool);

    double nlp_solver_step_length = 1;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "step_length", &nlp_solver_step_length);

    double levenberg_marquardt = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "levenberg_marquardt", &levenberg_marquardt);

    /* options QP solver */

    int qp_solver_iter_max = 50;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "qp_iter_max", &qp_solver_iter_max);
    // set SQP specific options
    double nlp_solver_tol_stat = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "tol_stat", &nlp_solver_tol_stat);

    double nlp_solver_tol_eq = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "tol_eq", &nlp_solver_tol_eq);

    double nlp_solver_tol_ineq = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "tol_ineq", &nlp_solver_tol_ineq);

    double nlp_solver_tol_comp = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "tol_comp", &nlp_solver_tol_comp);

    int nlp_solver_max_iter = 200;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "max_iter", &nlp_solver_max_iter);

    int initialize_t_slacks = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "initialize_t_slacks", &initialize_t_slacks);

    int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "print_level", &print_level);


    int ext_cost_num_hess = 0;


    /* out */
    ocp_nlp_out * nlp_out = ocp_nlp_out_create(nlp_config, nlp_dims);
    capsule->nlp_out = nlp_out;

    // initialize primal solution
    double x0[10];

    // initialize with x0
    
    x0[0] = 0;
    x0[1] = 0;
    x0[2] = 0;
    x0[3] = 1;
    x0[4] = 0;
    x0[5] = 0;
    x0[6] = 0;
    x0[7] = 0;
    x0[8] = 0;
    x0[9] = 0;


    double u0[NU];
    
    u0[0] = 0.0;
    u0[1] = 0.0;
    u0[2] = 0.0;
    u0[3] = 0.0;

    for (int i = 0; i < N; i++)
    {
        // x0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x0);
        // u0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x0);
    
    capsule->nlp_solver = ocp_nlp_solver_create(nlp_config, nlp_dims, capsule->nlp_opts);




    status = ocp_nlp_precompute(capsule->nlp_solver, nlp_in, nlp_out);

    if (status != ACADOS_SUCCESS)
    {
        printf("\nocp_precompute failed!\n\n");
        exit(1);
    }

    return status;
}


int quadrotor_q_acados_update_params(nlp_solver_capsule * capsule, int stage, double *p, int np)
{
    int solver_status = 0;

    int casadi_np = 0;
    if (casadi_np != np) {
        printf("acados_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }

    return solver_status;
}



int quadrotor_q_acados_solve(nlp_solver_capsule * capsule)
{
    // solve NLP 
    int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}


int quadrotor_q_acados_free(nlp_solver_capsule * capsule)
{
    // free memory
    ocp_nlp_solver_opts_destroy(capsule->nlp_opts);
    ocp_nlp_in_destroy(capsule->nlp_in);
    ocp_nlp_out_destroy(capsule->nlp_out);
    ocp_nlp_solver_destroy(capsule->nlp_solver);
    ocp_nlp_dims_destroy(capsule->nlp_dims);
    ocp_nlp_config_destroy(capsule->nlp_config);
    ocp_nlp_plan_destroy(capsule->nlp_solver_plan);

    /* free external function */
    // dynamics
    for (int i = 0; i < 20; i++)
    {
        external_function_param_casadi_free(&capsule->forw_vde_casadi[i]);
        external_function_param_casadi_free(&capsule->expl_ode_fun[i]);
    }
    free(capsule->forw_vde_casadi);
    free(capsule->expl_ode_fun);

    // cost

    // constraints

    return 0;
}

ocp_nlp_in *quadrotor_q_acados_get_nlp_in(nlp_solver_capsule * capsule) { return capsule->nlp_in; }
ocp_nlp_out *quadrotor_q_acados_get_nlp_out(nlp_solver_capsule * capsule) { return capsule->nlp_out; }
ocp_nlp_solver *quadrotor_q_acados_get_nlp_solver(nlp_solver_capsule * capsule) { return capsule->nlp_solver; }
ocp_nlp_config *quadrotor_q_acados_get_nlp_config(nlp_solver_capsule * capsule) { return capsule->nlp_config; }
void *quadrotor_q_acados_get_nlp_opts(nlp_solver_capsule * capsule) { return capsule->nlp_opts; }
ocp_nlp_dims *quadrotor_q_acados_get_nlp_dims(nlp_solver_capsule * capsule) { return capsule->nlp_dims; }
ocp_nlp_plan *quadrotor_q_acados_get_nlp_plan(nlp_solver_capsule * capsule) { return capsule->nlp_solver_plan; }


void quadrotor_q_acados_print_stats(nlp_solver_capsule * capsule)
{
    int sqp_iter, stat_m, stat_n, tmp_int;
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "sqp_iter", &sqp_iter);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_n", &stat_n);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_m", &stat_m);

    
    double stat[2000];
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "statistics", stat);

    int nrow = sqp_iter+1 < stat_m ? sqp_iter+1 : stat_m;

    printf("iter\tres_stat\tres_eq\t\tres_ineq\tres_comp\tqp_stat\tqp_iter\n");
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < stat_n + 1; j++)
        {
            if (j == 0 || j > 4)
            {
                tmp_int = (int) stat[i + j * nrow];
                printf("%d\t", tmp_int);
            }
            else
            {
                printf("%e\t", stat[i + j * nrow]);
            }
        }
        printf("\n");
    }
}

