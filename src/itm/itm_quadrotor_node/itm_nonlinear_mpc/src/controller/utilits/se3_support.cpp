/*
 * @Author: Wei Luo
 * @Date: 2021-07-05 12:32:41
 * @LastEditors: Wei Luo
 * @LastEditTime: 2021-07-10 22:04:10
 * @Note: Note
 */

#include <itm_nonlinear_mpc/controller/utilitis/se3_support.hpp>

template <typename T>
int signum(T val)
{
    return (T(0) < val) - (val < T(0));
}

double getHeading(geometry_msgs::Quaternion q)
{
    tf2::Vector3 b1 = tf2::Vector3(1, 0, 0);

    tf2::convert(q, tf2_quaternion_);

    tf2::Vector3 x_new = tf2::Transform(tf2_quaternion_) * b1;

    if (fabs(x_new[0]) <= 1e-3 && fabs(x_new[1]) <= 1e-3)
    {
        ROS_ERROR("error by getting the heading");
    }

    return atan2(x_new[1], x_new[0]);
}

double getHeadingRate(const geometry_msgs::Quaternion q, const Eigen::Vector3d yaw_array)
{
    Eigen::Matrix3d R = geom_q_to_rotation(q);
    // create the angular velocity tensor
    Eigen::Matrix3d W;
    W << 0, -yaw_array[2], yaw_array[1], yaw_array[2], 0, -yaw_array[0], -yaw_array[1], yaw_array[0], 0;

    // R derivative
    Eigen::Matrix3d R_d = R * W;

    // atan2 derivative
    double rx = R(0, 0); // x-component of body X
    double ry = R(1, 0); // y-component of body Y

    double denom = rx * rx + ry * ry;

    if (fabs(denom) <= 1e-5)
    {
        ROS_ERROR("[AttitudeConverter]: getHeadingRate(): denominator near zero!!!");
    }

    double atan2_d_x = -ry / denom;
    double atan2_d_y = rx / denom;

    // atan2 total differential
    double heading_rate = atan2_d_x * R_d(0, 0) + atan2_d_y * R_d(1, 0);

    return heading_rate;
}

double getYawRateIntrinsic(const geometry_msgs::Quaternion q, const double &heading_rate)
{
    // when the heading rate is very small, it does not make sense to compute the
    // yaw rate (the math would break), return 0
    if (fabs(heading_rate) < 1e-3)
    {
        return 0;
    }
    Eigen::Matrix3d R = geom_q_to_rotation(q);

    // construct the heading orbital velocity vector
    Eigen::Vector3d heading_vector = Eigen::Vector3d(R(0, 0), R(1, 0), 0);
    Eigen::Vector3d orbital_velocity = Eigen::Vector3d(0, 0, heading_rate).cross(heading_vector);

    // projector to the heading orbital velocity vector subspace
    Eigen::Vector3d b_orb = Eigen::Vector3d(0, 0, 1).cross(heading_vector);
    b_orb.normalize();
    Eigen::Matrix3d P = b_orb * b_orb.transpose();

    // project the body yaw orbital velocity vector base onto the heading orbital velocity vector subspace
    Eigen::Vector3d projected = P * R.col(1);

    double orbital_velocity_norm = orbital_velocity.norm();
    double projected_norm = projected.norm();

    if (fabs(projected_norm) < 1e-5)
    {
        ROS_ERROR("[AttitudeConverter]: getYawRateIntrinsic(): \"projected_norm\" in denominator is almost zero!!!");
    }

    double direction = signum(orbital_velocity.dot(projected));

    double output_yaw_rate = direction * (orbital_velocity_norm / projected_norm);

    if (!std::isfinite(output_yaw_rate))
    {
        ROS_ERROR("[AttitudeConverter]: getYawRateIntrinsic(): NaN detected in variable \"output_yaw_rate\"!!!");
    }

    // extract the yaw rate
    return output_yaw_rate;
}

Eigen::Matrix3d setHeadingByYaw(double heading, Eigen::Matrix3d rd)
{
    Eigen::Vector3d b1 = rd * Eigen::Vector3d(1, 0, 0);
    Eigen::Vector3d b3 = rd * Eigen::Vector3d(0, 0, 1);

    if (fabs(b3[2]) < 1e-3)
    {
        ROS_ERROR("Yaw angle setup error");
    }

    Eigen::Vector3d h(cos(heading), sin(heading), 0.0);
    // cast down the heading vector to the plane orthogonal to b3 (the thrust vector)
    Eigen::Vector3d heading_vec_sklop(h[0], h[1], (-b3[0] * h[0] - b3[1] * h[1]) / b3[2]);
    double yaw_diff = angleBetween(b1, heading_vec_sklop);
    // get the rotation around b3 about yaw_diff
    Eigen::Matrix3d rotator = Eigen::AngleAxisd(-yaw_diff, b3).toRotationMatrix();

    // concatenate the transformations
    Eigen::Matrix3d new_attitude = rotator * rd;

    return new_attitude;
}

double angleBetween(const Eigen::Vector3d &vec1, const Eigen::Vector3d &vec2)
{
    const double sin_12 = vec1.cross(vec2).norm();
    const double cos_12 = vec1.dot(vec2);
    const double angle = std::atan2(sin_12, cos_12);
    return angle;
}

Eigen::Matrix3d geom_q_to_rotation(const geometry_msgs::Quaternion q)
{
    Eigen::Quaterniond quaternion(q.w, q.x, q.y, q.z);
    Eigen::Matrix3d R = quaternion.toRotationMatrix();
    return R;
}

geometry_msgs::Quaternion getQuaternionfromMatrix(const Eigen::Matrix3d rd)
{
    Eigen::Quaterniond quaternion = Eigen::Quaterniond(rd);
    geometry_msgs::Quaternion geo_quaternion;
    geo_quaternion.w = quaternion.w();
    geo_quaternion.x = quaternion.x();
    geo_quaternion.y = quaternion.y();
    geo_quaternion.z = quaternion.z();
    return geo_quaternion;
}

double forceToThrust(const double motor_parameter_a, const double motor_parameter_b, const double force, const int num_motors)
{
    return sqrt(force / num_motors) * motor_parameter_a + motor_parameter_b;
}