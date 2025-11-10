//
// Created by Daniel Schatz on 07.11.25.
//

#include "../self/eulerSolver.h"

EulerSolver::EulerSolver() {
}

EulerSolver::~EulerSolver() {
}

void EulerSolver::start(SystemState *initial, double dt) {
    Solver::start(initial, dt);
}

bool EulerSolver::step(SystemState *system) {
    system->dt = m_dt;
    return true;
}

void EulerSolver::solve(SystemState *system) {
    system->dt = m_dt;

    for (int i = 0; i < system->n; ++i) {
        system->theta_x[i] += m_dt * system->v_theta_x[i];
        system->theta_y[i] += m_dt * system->v_theta_y[i];
        system->theta_z[i] += m_dt * system->v_theta_z[i];
        system->v_theta_x[i] += m_dt * system->a_theta_x[i];
        system->v_theta_y[i] += m_dt * system->a_theta_y[i];
        system->v_theta_z[i] += m_dt * system->a_theta_z[i];
        system->p_x[i] += m_dt * system->v_x[i];
        system->p_y[i] += m_dt * system->v_y[i];
        system->p_z[i] += m_dt * system->v_z[i];
        system->v_x[i] += m_dt * system->a_x[i];
        system->v_y[i] += m_dt * system->a_y[i];
        system->v_z[i] += m_dt * system->a_z[i];
    }
}

void EulerSolver::end() {
}