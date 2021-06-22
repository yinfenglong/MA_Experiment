
#include <acado_toolkit.hpp>

int main (int argc, char * const argv[ ])
{
    USING_NAMESPACE_ACADO
    using namespace std;

    double g = 9.8066;
    double PI = 3.1415926535897932;

    /* Time horizon */
    double Ts = 0.3;  // prediction sampling time 0.1
    double N  = 20;   // Prediction horizon 20

    /* Differential states */
    DifferentialState position1; //x_w
    DifferentialState position2; //y_w
    DifferentialState position3; //z_w
    DifferentialState velocity1; //velocity x_w
    DifferentialState velocity2; //velocity y_w
    DifferentialState velocity3; //velocity z_w
    DifferentialState qw;
    DifferentialState qx;
    DifferentialState qy;
    DifferentialState qz;

    /* Controls (inputs) in shape 4 */
    Control roll_rate_ref;
    Control pitch_rate_ref;
    Control yaw_rate_ref;
    Control thrust;

    /* OnlineData */
    OnlineData external_forces1;
    OnlineData external_forces2;
    OnlineData external_forces3;


    /* Differential equation (system definition external_forces) */
    DifferentialEquation f;

    f << dot( position1 ) == velocity1;
    f << dot( position2 ) == velocity2;
    f << dot( position3 ) == velocity3;
    f << dot( velocity1 ) == 2.0*(qw*qy + qx*qz) * thrust + external_forces1;
    f << dot(velocity2)   == 2.0*(qy*qz - qw*qx) * thrust + external_forces2;
    f << dot(velocity3)   == -g + (qw*qw-qx*qx-qy*qy+qz*qz) * thrust + external_forces3;
    f << dot( qw )        == 0.5*(-roll_rate_ref*qx-pitch_rate_ref*qy-yaw_rate_ref*qz);
    f << dot( qx )     == 0.5*(roll_rate_ref*qw-pitch_rate_ref*qz+yaw_rate_ref*qy);
    f << dot( qy )     == 0.5*(roll_rate_ref*qz+pitch_rate_ref*qw-yaw_rate_ref*qx);
    f << dot( qz )     == 0.5*(-roll_rate_ref*qy+pitch_rate_ref*qx+yaw_rate_ref*qw);

    /* Objective function
     (Function h for ocp.minimizeLSQ, hN for ocp.minimizeLSQEndTerm)
    */
    Function h;
    h << position1 << position2 << position3;
    h << velocity1 << velocity2 << velocity3;
    h << qw << qx << qy << qz;
    h << roll_rate_ref  << pitch_rate_ref << yaw_rate_ref;
    h << (qw*qw-qx*qx-qy*qy+qz*qz)*thrust - g;

    Function hN;
    hN << position1 << position2 << position3;
    hN << velocity1 << velocity2 << velocity3;

    BMatrix W  = eye<bool>(h.getDim());                 //weighting matrixs
    BMatrix WN = eye<bool>(hN.getDim());                //weighting matrixs of EndTerm

    /* Define OCP problem */
    OCP ocp(0.0, N*Ts, N);
    // ocp.subjectTo(f);

    ocp.minimizeLSQ(W, h);
    ocp.minimizeLSQEndTerm(WN, hN);

    /* Constraints, real ones set online */
    ocp.subjectTo( f );
    ocp.subjectTo(-90*PI/180 <= roll_rate_ref  <= 90*PI/180);
    ocp.subjectTo(-90*PI/180 <= pitch_rate_ref <= 90*PI/180);
    ocp.subjectTo(-30*PI/180 <= yaw_rate_ref <= 30*PI/180);
    ocp.subjectTo(     g/2.0 <= thrust    <= g*1.5);

    /* Export the code */
    OCPexport mpc( ocp );

    mpc.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    mpc.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    mpc.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2); //FULL_CONDENsinG_N2
    mpc.set( INTEGRATOR_TYPE, INT_IRK_GL2);
    //mpc.set( NUM_INTEGRATOR_STEPS, N);
    mpc.set( QP_SOLVER, QP_QPOASES);
    mpc.set( HOTSTART_QP, NO);
    mpc.set( LEVENBERG_MARQUARDT, 1e-10);
    mpc.set( LINEAR_ALGEBRA_SOLVER, GAUSS_LU);
    mpc.set( IMPLICIT_INTEGRATOR_NUM_ITS, 2);
    mpc.set( CG_USE_OPENMP, YES);
    mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO);
    mpc.set( CG_USE_VARIABLE_WEIGHTING_MATRIX, NO);

    if (mpc.exportCode( "../execute_code_uav_rate" ) != SUCCESSFUL_RETURN)
            exit( EXIT_FAILURE );

    mpc.printDimensionsQP();

    return EXIT_SUCCESS;
}