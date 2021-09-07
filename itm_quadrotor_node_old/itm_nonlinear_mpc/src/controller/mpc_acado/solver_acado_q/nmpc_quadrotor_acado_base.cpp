/*
 * @Author: Wei Luo
 * @Date: 2021-05-28 00:49:14
 * @LastEditors: Wei Luo
 * @LastEditTime: 2021-05-28 12:26:58
 * @Note: Note
 */

#include <acado_toolkit.hpp>

int main(int argc, char *const argv[])
{
    USING_NAMESPACE_ACADO
    // using namespace std;

    double g_ = 9.8066;
    double PI = 3.1415926535897932;

    /* Time horizon */
    double Ts = 0.3; // prediction sampling time 0.1
    double N = 20;   // Prediction horizon 20

    /* Differential states */
    DifferentialState velocity1; //velocity x_w
    DifferentialState velocity2; //velocity y_w
    DifferentialState velocity3; //velocity z_w
    DifferentialState qw;
    DifferentialState qx;
    DifferentialState qy;
    DifferentialState qz;
    DifferentialState position1; //x_w
    DifferentialState position2; //y_w
    DifferentialState position3; //z_w

    /* Controls (inputs) */
    Control roll_rate_des;
    Control pitch_rate_des;
    Control yaw_rate_des;
    Control thrust;

    /* Differential equation */
    DifferentialEquation f;

    f << dot(position1) == velocity1;
    f << dot(position2) == velocity2;
    f << dot(position3) == velocity3;
    f << dot(velocity1) == 2 * (qw * qy + qx * qz) * thrust;
    f << dot(velocity2) == 2 * (qy * qz - qw * qx) * thrust;
    f << dot(velocity3) == (qw * qw - qx * qx - qy * qy + qz * qz) * thrust - g_;
    f << dot(qw) == 0.5 * (-roll_rate_des * qx - pitch_rate_des * qy - yaw_rate_des * qz);
    f << dot(qx) == 0.5 * (roll_rate_des * qw + yaw_rate_des * qy - pitch_rate_des * qz);
    f << dot(qy) == 0.5 * (pitch_rate_des * qw - yaw_rate_des * qx + roll_rate_des * qz);
    f << dot(qz) == 0.5 * (yaw_rate_des * qw + pitch_rate_des * qx - roll_rate_des * qy);

    /* objective function */
    Function h;

    h << position1 << position2 << position3;
    h << velocity1 << velocity2 << velocity3;
    h << qw << qx << qy << qz;
    h << roll_rate_des << pitch_rate_des << yaw_rate_des << thrust;

    Function hN;

    hN << position1 << position2 << position3;
    hN << velocity1 << velocity2 << velocity3;

    BMatrix W = std::eye<bool>(h.getDim());
    BMatrix WN = std::eye<bool>(hN.getDim());
}