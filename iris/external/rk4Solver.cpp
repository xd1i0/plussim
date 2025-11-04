//
// Created by Daniel Schatz on 04.11.25.
//

#include "rk4Solver.h"

Rk4Solver::Rk4Solver() {
    m_stage = m_nextStage = RkStage::Undefined;
}

Rk4Solver::~Rk4Solver() {
    m_initialState.destroy();
    m_accumulator.destroy();
}

void Rk4Solver::start(SystemState *initial, double dt) {
    Solver::start(initial, dt);

    m_initialState.copy(initial);
    m_accumulator.copy(initial);

    m_stage = RkStage::Stage_1;
}

bool Rk4Solver::step(SystemState *state) {
    switch (m_stage) {
        case RkStage::Stage_1:
            state->dt = 0.0;
            break;
        case RkStage::Stage_2:
        case RkStage::Stage_3:
            for (int i = 0; i < state->n; ++i) {
                state->v_theta_x[i] =
                    m_initialState.v_theta_x[i] + m_dt * state->a_theta_x[i] / 2.0;
                state->v_theta_y[i] =
                    m_initialState.v_theta_y[i] + m_dt * state->a_theta_y[i] / 2.0;
                state->v_theta_z[i] =
                    m_initialState.v_theta_z[i] + m_dt * state->a_theta_z[i] / 2.0;
                state->theta_x[i] =
                    m_initialState.theta_x[i] + m_dt * state->v_theta_x[i] / 2.0;
                state->theta_y[i] =
                    m_initialState.theta_y[i] + m_dt * state->v_theta_y[i] / 2.0;
                state->theta_z[i] =
                    m_initialState.theta_z[i] + m_dt * state->v_theta_z[i] / 2.0;
                state->v_x[i] =
                    m_initialState.v_x[i] + m_dt * state->a_x[i] / 2.0;
                state->v_y[i] =
                    m_initialState.v_y[i] + m_dt * state->a_y[i] / 2.0;
                state->v_z[i] =
                    m_initialState.v_z[i] + m_dt * state->a_z[i] / 2.0;
                state->p_x[i] =
                    m_initialState.p_x[i] + m_dt * state->v_x[i] / 2.0;
                state->p_y[i] =
                    m_initialState.p_y[i] + m_dt * state->v_y[i] / 2.0;
                state->p_z[i] =
                    m_initialState.p_z[i] + m_dt * state->v_z[i] / 2.0;
            }

            state->dt = m_dt / 2.0;
            break;
        case RkStage::Stage_4:
            for (int i = 0; i < state->n; ++i) {
                state->v_theta_x[i] =
                    m_initialState.v_theta_x[i] + m_dt * state->a_theta_x[i];
                state->v_theta_y[i] =
                    m_initialState.v_theta_y[i] + m_dt * state->a_theta_y[i];
                state->v_theta_z[i] =
                    m_initialState.v_theta_z[i] + m_dt * state->a_theta_z[i];
                state->theta_x[i] =
                    m_initialState.theta_x[i] + m_dt * state->v_theta_x[i];
                state->theta_y[i] =
                    m_initialState.theta_y[i] + m_dt * state->v_theta_y[i];
                state->theta_z[i] =
                    m_initialState.theta_z[i] + m_dt * state->v_theta_z[i];
                state->v_x[i] =
                    m_initialState.v_x[i] + m_dt * state->a_x[i];
                state->v_y[i] =
                    m_initialState.v_y[i] + m_dt * state->a_y[i];
                state->v_z[i] =
                    m_initialState.v_z[i] + m_dt * state->a_z[i];
                state->p_x[i] =
                    m_initialState.p_x[i] + m_dt * state->v_x[i];
                state->p_y[i] =
                    m_initialState.p_y[i] + m_dt * state->v_y[i];
                state->p_z[i] =
                    m_initialState.p_z[i] + m_dt * state->v_z[i];
            }

            state->dt = m_dt;
            break;
        default:
            break;
    }

    m_nextStage = getNextStage(m_stage);

    return m_nextStage == RkStage::Complete;
}

void Rk4Solver::solve(SystemState *system) {
    double stageWeight = 0.0;
    switch (m_stage) {
        case RkStage::Stage_1: stageWeight = 1.0; break;
        case RkStage::Stage_2: stageWeight = 2.0; break;
        case RkStage::Stage_3: stageWeight = 2.0; break;
        case RkStage::Stage_4: stageWeight = 1.0; break;
        default: stageWeight = 0.0;
    }

    for (int i = 0; i < system->n; ++i) {
        m_accumulator.v_theta_x[i] += (m_dt / 6.0) * system->a_theta_x[i] * stageWeight;
        m_accumulator.v_theta_y[i] += (m_dt / 6.0) * system->a_theta_y[i] * stageWeight;
        m_accumulator.v_theta_z[i] += (m_dt / 6.0) * system->a_theta_z[i] * stageWeight;
        m_accumulator.theta_x[i] += (m_dt / 6.0) * system->v_theta_x[i] * stageWeight;
        m_accumulator.theta_y[i] += (m_dt / 6.0) * system->v_theta_y[i] * stageWeight;
        m_accumulator.theta_z[i] += (m_dt / 6.0) * system->v_theta_z[i] * stageWeight;
        m_accumulator.v_x[i] += (m_dt / 6.0) * system->a_x[i] * stageWeight;
        m_accumulator.v_y[i] += (m_dt / 6.0) * system->a_y[i] * stageWeight;
        m_accumulator.v_z[i] += (m_dt / 6.0) * system->a_z[i] * stageWeight;
        m_accumulator.p_x[i] += (m_dt / 6.0) * system->v_x[i] * stageWeight;
        m_accumulator.p_y[i] += (m_dt / 6.0) * system->v_y[i] * stageWeight;
        m_accumulator.p_z[i] += (m_dt / 6.0) * system->v_z[i] * stageWeight;
    }

    for (int i = 0; i < system->n_c; ++i) {
        m_accumulator.r_x[i] += (m_dt / 6.0) * system->r_x[i] * stageWeight;
        m_accumulator.r_y[i] += (m_dt / 6.0) * system->r_y[i] * stageWeight;
        m_accumulator.r_z[i] += (m_dt / 6.0) * system->r_z[i] * stageWeight;
        m_accumulator.r_t_x[i] += (m_dt / 6.0) * system->r_t_x[i] * stageWeight;
        m_accumulator.r_t_y[i] += (m_dt / 6.0) * system->r_t_y[i] * stageWeight;
        m_accumulator.r_t_z[i] += (m_dt / 6.0) * system->r_t_z[i] * stageWeight;
    }

    if (m_stage == RkStage::Stage_4) {
        for (int i = 0; i < system->n; ++i) {
            system->v_theta_x[i] = m_accumulator.v_theta_x[i];
            system->v_theta_y[i] = m_accumulator.v_theta_y[i];
            system->v_theta_z[i] = m_accumulator.v_theta_z[i];
            system->theta_x[i] = m_accumulator.theta_x[i];
            system->theta_y[i] = m_accumulator.theta_y[i];
            system->theta_z[i] = m_accumulator.theta_z[i];
            system->v_x[i] = m_accumulator.v_x[i];
            system->v_y[i] = m_accumulator.v_y[i];
            system->v_z[i] = m_accumulator.v_z[i];
            system->p_x[i] = m_accumulator.p_x[i];
            system->p_y[i] = m_accumulator.p_y[i];
            system->p_z[i] = m_accumulator.p_z[i];
        }

        for (int i = 0; i < system->n_c; ++i) {
            system->r_x[i] = m_accumulator.r_x[i];
            system->r_y[i] = m_accumulator.r_y[i];
            system->r_z[i] = m_accumulator.r_z[i];
            system->r_t_x[i] = m_accumulator.r_t_x[i];
            system->r_t_y[i] = m_accumulator.r_t_y[i];
            system->r_t_z[i] = m_accumulator.r_t_z[i];
        }
    }

    m_stage = m_nextStage;
}

void Rk4Solver::end() {
    Solver::end();

    m_stage = m_nextStage = RkStage::Undefined;
}

Rk4Solver::RkStage Rk4Solver::getNextStage(RkStage stage) {
    switch (stage) {
        case RkStage::Stage_1: return RkStage::Stage_2;
        case RkStage::Stage_2: return RkStage::Stage_3;
        case RkStage::Stage_3: return RkStage::Stage_4;
        case RkStage::Stage_4: return RkStage::Complete;
        default: return RkStage::Undefined;
    }
}
