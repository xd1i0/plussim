//
// Created by Daniel Schatz on 04.11.25.
//

#include "../self/solver.h"

Solver::Solver() {
    m_dt = 0.0;
}

Solver::~Solver() {
    /* void */
}

void Solver::start(SystemState *initial, double dt) {
    m_dt = dt;
}

bool Solver::step(SystemState *system) {
    return true;
}

void Solver::solve(SystemState *system) {
    /* void */
}

void Solver::end() {
    /* void */
}