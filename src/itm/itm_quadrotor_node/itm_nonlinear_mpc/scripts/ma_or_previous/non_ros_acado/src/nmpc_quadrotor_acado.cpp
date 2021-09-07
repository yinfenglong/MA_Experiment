
#include <acado_toolkit.hpp>

int main (int argc, char * const argv[ ])
{
    USING_NAMESPACE_ACADO
    using namespace std;

    double g = 9.8066;
    double PI = 3.1415926535897932;

    /* Time horizon */
    double Ts = 0.3; //0.3;  // prediction sampling time 0.1 0.5
    double N  = 20;   // Prediction horizon 20

    /* Differential states */
    DifferentialState position1; //x_w
    DifferentialState position2; //y_w
    DifferentialState position3; //z_w
    DifferentialState velocity1; //velocity x_w
    DifferentialState velocity2; //velocity y_w
    DifferentialState velocity3; //velocity z_w
    DifferentialState roll;
    DifferentialState pitch;
    DifferentialState yaw;

    /* Controls (inputs) */
    Control roll_ref;
    Control pitch_ref;
    Control thrust;

    /* OnlineData */
    OnlineData roll_tau;
    OnlineData roll_gain;
    OnlineData pitch_tau;
    OnlineData pitch_gain;
    // OnlineData linear_drag_coefficient1;
    // OnlineData linear_drag_coefficient2;

    // const double roll_tau = 0.257; // ETH
    // const double roll_gain = 0.75; // ETH
    // const double pitch_tau = 0.259;
    // const double pitch_gain = 0.78;
    // const double roll_tau = 0.47; // ETH
    // const double roll_gain = 0.75; // ETH
    // const double pitch_tau = 0.47;
    // const double pitch_gain = 0.78;
    // const double roll_tau = 0.477; // Bebop 1
    // const double roll_gain = 1.277; // Bebop 1
    // const double pitch_tau = 0.477; // Bebop 1
    // const double pitch_gain = 1.277; // Bebop 1
    const double linear_drag_coefficient1 = 0.01;
    const double linear_drag_coefficient2 = 0.01;
    OnlineData external_forces1;
    OnlineData external_forces2;
    OnlineData external_forces3;


    /* IntermediateState (Non-linear drag) */
    IntermediateState dragacc1 =   sin(pitch)*linear_drag_coefficient1*thrust*velocity3
                                 + cos(pitch)*cos(yaw)*linear_drag_coefficient1*thrust*velocity1
                                 - cos(pitch)*linear_drag_coefficient1*sin(yaw)*thrust*velocity2;
    IntermediateState dragacc2 =   (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*linear_drag_coefficient2*thrust*velocity1
                                 - (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*linear_drag_coefficient2*thrust*velocity2
                                 - cos(pitch)*linear_drag_coefficient2*sin(roll)*thrust*velocity3;

    /* Differential equation (system definition external_forces) */
    DifferentialEquation f;

    f << dot( position1 ) == velocity1;
    f << dot( position2 ) == velocity2;
    f << dot( position3 ) == velocity3;
    f << dot(velocity1)   == ((cos(roll)*cos(yaw)*sin(pitch) + sin(roll)*sin(yaw))*thrust - dragacc1+external_forces1);
    f << dot(velocity2)   == ((cos(roll)*sin(pitch)*sin(yaw) - cos(yaw)*sin(roll))*thrust - dragacc2+external_forces2);
    f << dot(velocity3)   == (-g + cos(pitch)*cos(roll)*thrust+external_forces3);
    f << dot( roll )      == (roll_gain*roll_ref - roll)/roll_tau;
    f << dot( pitch )     == (pitch_gain*pitch_ref - pitch)/pitch_tau;
    f << dot( yaw )       == 0;

    /* Objective function
     (Function h for ocp.minimizeLSQ, hN for ocp.minimizeLSQEndTerm)
    */
    Function h;
    h << position1 << position2 << position3;
    h << velocity1 << velocity2 << velocity3;
    h << roll      << pitch;
    h << roll_ref  << pitch_ref << (cos(pitch)*cos(roll)*thrust - g);

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
    ocp.subjectTo(-45*PI/180 <= roll_ref  <= 45*PI/180);
    ocp.subjectTo(-45*PI/180 <= pitch_ref <= 45*PI/180);
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

    if (mpc.exportCode( "../execute_code_uav" ) != SUCCESSFUL_RETURN)
            exit( EXIT_FAILURE );

    mpc.printDimensionsQP();

    return EXIT_SUCCESS;
}
