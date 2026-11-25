#include "stellaris/system_state.h"

#include <cmath>
#include <cstddef>

void SystemState::copy(const SystemState *state) {
    *this = *state;
}

void SystemState::resize(int bodyCount, int constraintCount) {
    n = bodyCount;
    n_c = constraintCount;

    for (auto *v : {&a_theta_x, &a_theta_y, &a_theta_z,
                    &v_theta_x, &v_theta_y, &v_theta_z,
                    &theta_x, &theta_y, &theta_z,
                    &a_x, &a_y, &a_z,
                    &v_x, &v_y, &v_z,
                    &p_x, &p_y, &p_z,
                    &f_x, &f_y, &f_z,
                    &t_x, &t_y, &t_z,
                    &m}) {
        v->resize(n);
    }

    indexMap.resize(n_c);
    for (auto *v : {&r_x, &r_y, &r_z, &r_t_x, &r_t_y, &r_t_z}) {
        v->resize(static_cast<size_t>(n_c) * 2);
    }
}

void SystemState::localToWorld(
        double x,
        double y,
        double z,
        double *x_t,
        double *y_t,
        double *z_t,
        int body)
{
    const double x0 = p_x[body];
    const double y0 = p_y[body];
    const double z0 = p_z[body];

    const double theta_x = this->theta_x[body];
    const double theta_y = this->theta_y[body];
    const double theta_z = this->theta_z[body];

    // Rotation matrices for each axis (ZYX Euler angles)
    const double cos_x = std::cos(theta_x);
    const double sin_x = std::sin(theta_x);
    const double cos_y = std::cos(theta_y);
    const double sin_y = std::sin(theta_y);
    const double cos_z = std::cos(theta_z);
    const double sin_z = std::sin(theta_z);

    // Combined rotation matrix (ZYX order)
    const double r00 = cos_y * cos_z;
    const double r01 = sin_x * sin_y * cos_z - cos_x * sin_z;
    const double r02 = cos_x * sin_y * cos_z + sin_x * sin_z;
    const double r10 = cos_y * sin_z;
    const double r11 = sin_x * sin_y * sin_z + cos_x * cos_z;
    const double r12 = cos_x * sin_y * sin_z - sin_x * cos_z;
    const double r20 = -sin_y;
    const double r21 = sin_x * cos_y;
    const double r22 = cos_x * cos_y;

    *x_t = r00 * x + r01 * y + r02 * z + x0;
    *y_t = r10 * x + r11 * y + r12 * z + y0;
    *z_t = r20 * x + r21 * y + r22 * z + z0;
}

void SystemState::velocityAtPoint(
        double x,
        double y,
        double z,
        double *v_x,
        double *v_y,
        double *v_z,
        int body)
{
    double w_x, w_y, w_z;
    localToWorld(x, y, z, &w_x, &w_y, &w_z, body);

    // Angular velocity vector
    const double omega_x = this->v_theta_x[body];
    const double omega_y = this->v_theta_y[body];
    const double omega_z = this->v_theta_z[body];

    // Position relative to center of mass
    const double r_x = w_x - this->p_x[body];
    const double r_y = w_y - this->p_y[body];
    const double r_z = w_z - this->p_z[body];

    // Linear velocity = v_cm + omega x r
    const double angularToLinear_x = omega_y * r_z - omega_z * r_y;
    const double angularToLinear_y = omega_z * r_x - omega_x * r_z;
    const double angularToLinear_z = omega_x * r_y - omega_y * r_x;

    *v_x = this->v_x[body] + angularToLinear_x;
    *v_y = this->v_y[body] + angularToLinear_y;
    *v_z = this->v_z[body] + angularToLinear_z;
}

void SystemState::applyForce(
    double x_l,
    double y_l,
    double z_l,
    double f_x,
    double f_y,
    double f_z,
    int body)
{
    double w_x, w_y, w_z;
    localToWorld(x_l, y_l, z_l, &w_x, &w_y, &w_z, body);

    // Apply linear force
    this->f_x[body] += f_x;
    this->f_y[body] += f_y;
    this->f_z[body] += f_z;

    // Position relative to center of mass
    const double r_x = w_x - this->p_x[body];
    const double r_y = w_y - this->p_y[body];
    const double r_z = w_z - this->p_z[body];

    // Torque = r x F (cross product)
    this->t_x[body] += r_y * f_z - r_z * f_y;
    this->t_y[body] += r_z * f_x - r_x * f_z;
    this->t_z[body] += r_x * f_y - r_y * f_x;
}