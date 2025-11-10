#include "cube.h"
#include <cmath>

Cube::Cube(double mass, double x, double y, double z, double size)
    : m_size(size), m_initial_x(x), m_initial_y(y), m_initial_z(z),
      m_spring_enabled(false), m_spring_anchor_x(0.0), m_spring_anchor_y(0.0), m_spring_anchor_z(0.0),
      m_spring_k(0.0), m_spring_damping(0.0), m_spring_rest_length(0.0),
      m_solver(&m_euler_solver), m_solver_type(0)
{
    m_state.resize(1, 0);

    m_state.m[0] = mass;
    m_state.p_x[0] = x;
    m_state.p_y[0] = y;
    m_state.p_z[0] = z;

    m_state.v_x[0] = m_state.v_y[0] = m_state.v_z[0] = 0.0;
    m_state.v_theta_x[0] = m_state.v_theta_y[0] = m_state.v_theta_z[0] = 0.0;
    m_state.theta_x[0] = m_state.theta_y[0] = m_state.theta_z[0] = 0.0;
    m_state.a_x[0] = m_state.a_y[0] = m_state.a_z[0] = 0.0;
    m_state.a_theta_x[0] = m_state.a_theta_y[0] = m_state.a_theta_z[0] = 0.0;
    m_state.f_x[0] = m_state.f_y[0] = m_state.f_z[0] = 0.0;
    m_state.t_x[0] = m_state.t_y[0] = m_state.t_z[0] = 0.0;
}

Cube::~Cube() {
    m_state.destroy();
}

void Cube::update(double dt) {
    // Helper lambda to compute forces and accelerations
    auto computeForcesAndAccelerations = [this]() {
        m_state.f_x[0] = m_state.f_y[0] = m_state.f_z[0] = 0.0;
        m_state.t_x[0] = m_state.t_y[0] = m_state.t_z[0] = 0.0;

        m_state.applyForce(0.0, 0.0, 0.0, 0.0, -m_state.m[0] * GRAVITY, 0.0, 0);

        // Apply spring force if enabled
        if (m_spring_enabled) {
            double dx = m_spring_anchor_x - m_state.p_x[0];
            double dy = m_spring_anchor_y - m_state.p_y[0];
            double dz = m_spring_anchor_z - m_state.p_z[0];

            // Current spring length
            double current_length = std::sqrt(dx * dx + dy * dy + dz * dz);

            if (current_length > 1e-6) {  // Avoid division by zero
                double extension = current_length - m_spring_rest_length;

                double dir_x = dx / current_length;
                double dir_y = dy / current_length;
                double dir_z = dz / current_length;

                // Spring force magnitude (Hooke's law: F = k * extension)
                double spring_force = m_spring_k * extension;

                // Damping force (opposes velocity along spring direction)
                double vel_along_spring = m_state.v_x[0] * dir_x +
                                         m_state.v_y[0] * dir_y +
                                         m_state.v_z[0] * dir_z;
                double damping_force = -m_spring_damping * vel_along_spring;

                double total_force = spring_force + damping_force;

                m_state.applyForce(0.0, 0.0, 0.0,
                                 total_force * dir_x,
                                 total_force * dir_y,
                                 total_force * dir_z, 0);
            }
        }

        m_state.a_x[0] = m_state.f_x[0] / m_state.m[0];
        m_state.a_y[0] = m_state.f_y[0] / m_state.m[0];
        m_state.a_z[0] = m_state.f_z[0] / m_state.m[0];
    };

    m_solver->start(&m_state, dt);

    if (m_solver_type == 0) {
        computeForcesAndAccelerations();
        m_solver->solve(&m_state);
    } else {
        bool complete = false;
        do {
            complete = m_solver->step(&m_state);
            computeForcesAndAccelerations();
            m_solver->solve(&m_state);
        } while (!complete);
        m_solver->end();
    }
    if (m_state.p_y[0] <= m_size / 2.0) {
        m_state.p_y[0] = m_size / 2.0;
        if (m_state.v_y[0] < 0.0) {
            m_state.v_y[0] = 0.0;
        }
    }
}

void Cube::reset(double x, double y, double z) {
    m_initial_x = x;
    m_initial_y = y;
    m_initial_z = z;

    m_state.p_x[0] = x;
    m_state.p_y[0] = y;
    m_state.p_z[0] = z;

    m_state.v_x[0] = m_state.v_y[0] = m_state.v_z[0] = 0.0;
    m_state.v_theta_x[0] = m_state.v_theta_y[0] = m_state.v_theta_z[0] = 0.0;
    m_state.theta_x[0] = m_state.theta_y[0] = m_state.theta_z[0] = 0.0;
    m_state.a_x[0] = m_state.a_y[0] = m_state.a_z[0] = 0.0;
    m_state.a_theta_x[0] = m_state.a_theta_y[0] = m_state.a_theta_z[0] = 0.0;
    m_state.f_x[0] = m_state.f_y[0] = m_state.f_z[0] = 0.0;
    m_state.t_x[0] = m_state.t_y[0] = m_state.t_z[0] = 0.0;
}

void Cube::getPosition(double &x, double &y, double &z) const {
    x = m_state.p_x[0];
    y = m_state.p_y[0];
    z = m_state.p_z[0];
}

void Cube::setSpring(double anchor_x, double anchor_y, double anchor_z,
                     double spring_k, double damping, double rest_length) {
    m_spring_enabled = true;
    m_spring_anchor_x = anchor_x;
    m_spring_anchor_y = anchor_y;
    m_spring_anchor_z = anchor_z;
    m_spring_k = spring_k;
    m_spring_damping = damping;
    m_spring_rest_length = rest_length;
}

void Cube::getSpringAnchor(double &x, double &y, double &z) const {
    x = m_spring_anchor_x;
    y = m_spring_anchor_y;
    z = m_spring_anchor_z;
}

void Cube::setMass(double mass) {
    m_state.m[0] = mass;
}

void Cube::setSize(double size) {
    m_size = size;
}

void Cube::setSolverType(int type) {
    m_solver_type = type;
    if (type == 0) {
        m_solver = &m_euler_solver;
    } else if (type == 1) {
        m_solver = &m_rk4_solver;
    }
}
