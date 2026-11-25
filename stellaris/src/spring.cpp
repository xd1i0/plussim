//
// Created by Daniel Schatz on 25.11.26.
//

#include "stellaris/spring.h"
#include <cmath>

void Spring::apply(SystemState &state) {
    double dx = m_anchor_x - state.p_x[m_body];
    double dy = m_anchor_y - state.p_y[m_body];
    double dz = m_anchor_z - state.p_z[m_body];

    double current_length = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (current_length > 1e-6) {  // Avoid division by zero
        double extension = current_length - m_rest_length;

        double dir_x = dx / current_length;
        double dir_y = dy / current_length;
        double dir_z = dz / current_length;

        // Spring force magnitude (Hooke's law: F = k * extension)
        double spring_force = m_spring_k * extension;

        // Damping force (opposes velocity along spring direction)
        double vel_along_spring = state.v_x[m_body] * dir_x +
                                 state.v_y[m_body] * dir_y +
                                 state.v_z[m_body] * dir_z;
        double damping_force = -m_damping * vel_along_spring;

        double total_force = spring_force + damping_force;

        state.f_x[m_body] += total_force * dir_x;
        state.f_y[m_body] += total_force * dir_y;
        state.f_z[m_body] += total_force * dir_z;
    }

}