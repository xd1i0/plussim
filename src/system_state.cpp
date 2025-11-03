#include "system_state.h"

#include "utilities.h"

#include <assert.h>
#include <cstring>
#include <cmath>

atg_scs::SystemState::SystemState() {
    indexMap = nullptr;

    a_theta_x = nullptr;
    a_theta_y = nullptr;
    a_theta_z = nullptr;
    v_theta_x = nullptr;
    v_theta_y = nullptr;
    v_theta_z = nullptr;
    theta_x = nullptr;
    theta_y = nullptr;
    theta_z = nullptr;

    a_x = nullptr;
    a_y = nullptr;
    a_z = nullptr;
    v_x = nullptr;
    v_y = nullptr;
    v_z = nullptr;
    p_x = nullptr;
    p_y = nullptr;
    p_z = nullptr;

    f_x = nullptr;
    f_y = nullptr;
    f_z = nullptr;
    t_x = nullptr;
    t_y = nullptr;
    t_z = nullptr;

    m = nullptr;

    r_x = nullptr;
    r_y = nullptr;
    r_z = nullptr;
    r_t_x = nullptr;
    r_t_y = nullptr;
    r_t_z = nullptr;

    n = 0;
    n_c = 0;
    dt = 0.0;
}

atg_scs::SystemState::~SystemState() {
    assert(n == 0);
    assert(n_c == 0);
}

void atg_scs::SystemState::copy(const SystemState *state) {
    resize(state->n, state->n_c);

    if (state->n == 0) {
        return;
    }

    std::memcpy((void *)indexMap, (void *)state->indexMap, sizeof(int) * n_c);

    std::memcpy((void *)a_theta_x, (void *)state->a_theta_x, sizeof(double) * n);
    std::memcpy((void *)a_theta_y, (void *)state->a_theta_y, sizeof(double) * n);
    std::memcpy((void *)a_theta_z, (void *)state->a_theta_z, sizeof(double) * n);
    std::memcpy((void *)v_theta_x, (void *)state->v_theta_x, sizeof(double) * n);
    std::memcpy((void *)v_theta_y, (void *)state->v_theta_y, sizeof(double) * n);
    std::memcpy((void *)v_theta_z, (void *)state->v_theta_z, sizeof(double) * n);
    std::memcpy((void *)theta_x, (void *)state->theta_x, sizeof(double) * n);
    std::memcpy((void *)theta_y, (void *)state->theta_y, sizeof(double) * n);
    std::memcpy((void *)theta_z, (void *)state->theta_z, sizeof(double) * n);

    std::memcpy((void *)a_x, (void *)state->a_x, sizeof(double) * n);
    std::memcpy((void *)a_y, (void *)state->a_y, sizeof(double) * n);
    std::memcpy((void *)a_z, (void *)state->a_z, sizeof(double) * n);
    std::memcpy((void *)v_x, (void *)state->v_x, sizeof(double) * n);
    std::memcpy((void *)v_y, (void *)state->v_y, sizeof(double) * n);
    std::memcpy((void *)v_z, (void *)state->v_z, sizeof(double) * n);
    std::memcpy((void *)p_x, (void *)state->p_x, sizeof(double) * n);
    std::memcpy((void *)p_y, (void *)state->p_y, sizeof(double) * n);
    std::memcpy((void *)p_z, (void *)state->p_z, sizeof(double) * n);

    std::memcpy((void *)f_x, (void *)state->f_x, sizeof(double) * n);
    std::memcpy((void *)f_y, (void *)state->f_y, sizeof(double) * n);
    std::memcpy((void *)f_z, (void *)state->f_z, sizeof(double) * n);
    std::memcpy((void *)t_x, (void *)state->t_x, sizeof(double) * n);
    std::memcpy((void *)t_y, (void *)state->t_y, sizeof(double) * n);
    std::memcpy((void *)t_z, (void *)state->t_z, sizeof(double) * n);

    std::memcpy((void *)m, (void *)state->m, sizeof(double) * n);

    std::memcpy((void *)r_x, (void *)state->r_x, sizeof(double) * n_c * 2);
    std::memcpy((void *)r_y, (void *)state->r_y, sizeof(double) * n_c * 2);
    std::memcpy((void *)r_z, (void *)state->r_z, sizeof(double) * n_c * 2);
    std::memcpy((void *)r_t_x, (void *)state->r_t_x, sizeof(double) * n_c * 2);
    std::memcpy((void *)r_t_y, (void *)state->r_t_y, sizeof(double) * n_c * 2);
    std::memcpy((void *)r_t_z, (void *)state->r_t_z, sizeof(double) * n_c * 2);
}

void atg_scs::SystemState::resize(int bodyCount, int constraintCount) {
    if (n >= bodyCount && n_c >= constraintCount) {
        return;
    }

    destroy();

    n = bodyCount;
    n_c = constraintCount;

    indexMap = new int[n_c];

    a_theta_x = new double[n];
    a_theta_y = new double[n];
    a_theta_z = new double[n];
    v_theta_x = new double[n];
    v_theta_y = new double[n];
    v_theta_z = new double[n];
    theta_x = new double[n];
    theta_y = new double[n];
    theta_z = new double[n];

    a_x = new double[n];
    a_y = new double[n];
    a_z = new double[n];
    v_x = new double[n];
    v_y = new double[n];
    v_z = new double[n];
    p_x = new double[n];
    p_y = new double[n];
    p_z = new double[n];

    f_x = new double[n];
    f_y = new double[n];
    f_z = new double[n];
    t_x = new double[n];
    t_y = new double[n];
    t_z = new double[n];

    m = new double[n];

    r_x = new double[(size_t)n_c * 2];
    r_y = new double[(size_t)n_c * 2];
    r_z = new double[(size_t)n_c * 2];
    r_t_x = new double[(size_t)n_c * 2];
    r_t_y = new double[(size_t)n_c * 2];
    r_t_z = new double[(size_t)n_c * 2];
}

void atg_scs::SystemState::destroy() {
    if (n > 0) {
        freeArray(a_theta_x);
        freeArray(a_theta_y);
        freeArray(a_theta_z);
        freeArray(v_theta_x);
        freeArray(v_theta_y);
        freeArray(v_theta_z);
        freeArray(theta_x);
        freeArray(theta_y);
        freeArray(theta_z);

        freeArray(a_x);
        freeArray(a_y);
        freeArray(a_z);
        freeArray(v_x);
        freeArray(v_y);
        freeArray(v_z);
        freeArray(p_x);
        freeArray(p_y);
        freeArray(p_z);

        freeArray(f_x);
        freeArray(f_y);
        freeArray(f_z);
        freeArray(t_x);
        freeArray(t_y);
        freeArray(t_z);

        freeArray(m);
    }

    if (n_c > 0) {
        freeArray(indexMap);

        freeArray(r_x);
        freeArray(r_y);
        freeArray(r_z);
        freeArray(r_t_x);
        freeArray(r_t_y);
        freeArray(r_t_z);
    }

    n = 0;
    n_c = 0;
}

void atg_scs::SystemState::localToWorld(
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

void atg_scs::SystemState::velocityAtPoint(
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

void atg_scs::SystemState::applyForce(
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