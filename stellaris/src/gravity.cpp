//
// Created by Daniel Schatz on 25.11.26.
//
#include "stellaris/gravity.h"

void Gravity::apply(SystemState &state) {
    for (int i = 0; i < state.n; ++i) {
        state.f_y[i] -= state.m[i] * m_g;
    }
}
