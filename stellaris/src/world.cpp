//
// Created by Daniel Schatz on 25.11.26.
//

#include "stellaris/world.h"

#include <algorithm>
#include <stdexcept>

int World::addBody(const BodyDef &body) {
    if (body.mass <= 0.0) {
        throw std::invalid_argument("mass must be positive");
    }

    int bodyIndex = m_state.n;
    m_state.resize(m_state.n + 1, m_state.n_c);

    m_state.m[bodyIndex] = body.mass;
    m_state.p_x[bodyIndex] = body.x;
    m_state.p_y[bodyIndex] = body.y;
    m_state.p_z[bodyIndex] = body.z;
    m_state.v_x[bodyIndex] = body.v_x;
    m_state.v_y[bodyIndex] = body.v_y;
    m_state.v_z[bodyIndex] = body.v_z;

    return bodyIndex;
}

void World::computeAccelerations() {
    std::ranges::fill(m_state.f_x, 0.0);
    std::ranges::fill(m_state.f_y, 0.0);
    std::ranges::fill(m_state.f_z, 0.0);

    std::ranges::fill(m_state.t_x, 0.0);
    std::ranges::fill(m_state.t_y, 0.0);
    std::ranges::fill(m_state.t_z, 0.0);

    for (auto &force : m_forces) {
        force->apply(m_state);
    }

    for (int i = 0 ; i < m_state.n; ++i) {
        m_state.a_x[i] = m_state.f_x[i] / m_state.m[i];
        m_state.a_y[i] = m_state.f_y[i] / m_state.m[i];
        m_state.a_z[i] = m_state.f_z[i] / m_state.m[i];
    }
}

void World::step(double dt) {
    m_solver->start(&m_state, dt);

    bool complete = false;
    while (!complete) {
        complete = m_solver->step(&m_state);
        computeAccelerations();
        m_solver->solve(&m_state);
    }

    m_solver->end();
}
