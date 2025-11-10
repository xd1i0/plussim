//
// Created by Daniel Schatz on 02.11.25.
//

#include "double_pendulum.h"
#include "raylib.h"
#include <cmath>
#include "../external/self/rk4Solver.h"

DoublePendulum::DoublePendulum(float length1, float length2, float mass1, float mass2,
                               float angle1, float angle2, float timeStep)
    : l1(length1), l2(length2), m1(mass1), m2(mass2),
      a1(angle1), a2(angle2), dt(timeStep),
      av1(0.0f), av2(0.0f), aa1(0.0f), aa2(0.0f), g(9.81) {
}

void DoublePendulum::update() {
    // Use RK4 solver instead of the old explicit Euler integrator.
    // Map the two pendulum angles/velocities into a SystemState using the z-rotation
    // (theta_z / v_theta_z / a_theta_z) and run the Rk4Solver for one time step dt.

    SystemState state;
    state.resize(2, 0); // two bodies (two masses), no constraints

    // initialize arrays to zero and set masses
    for (int i = 0; i < state.n; ++i) {
        state.theta_x[i] = state.theta_y[i] = 0.0;
        state.v_theta_x[i] = state.v_theta_y[i] = 0.0;
        state.a_theta_x[i] = state.a_theta_y[i] = 0.0;

        state.theta_z[i] = 0.0;
        state.v_theta_z[i] = 0.0;
        state.a_theta_z[i] = 0.0;

        state.a_x[i] = state.a_y[i] = state.a_z[i] = 0.0;
        state.v_x[i] = state.v_y[i] = state.v_z[i] = 0.0;
        state.p_x[i] = state.p_y[i] = state.p_z[i] = 0.0;

        state.f_x[i] = state.f_y[i] = state.f_z[i] = 0.0;
        state.t_x[i] = state.t_y[i] = state.t_z[i] = 0.0;

        state.m[i] = (i == 0) ? m1 : m2;
    }

    // Map current pendulum state into the SystemState (use z-rotation)
    state.theta_z[0] = a1;
    state.theta_z[1] = a2;
    state.v_theta_z[0] = av1;
    state.v_theta_z[1] = av2;

    Rk4Solver solver;
    solver.start(&state, static_cast<double>(dt));

    bool complete = false;
    do {
        // Prepare the state for the current RK stage
        complete = solver.step(&state);

        // Compute accelerations for the current evaluation point using our helper
        std::array<double, 4> s = { state.theta_z[0], state.theta_z[1], state.v_theta_z[0], state.v_theta_z[1] };
        std::array<double, 4> ds;
        computeDerivatives(s, ds);

        // Put angular accelerations into the SystemState (z axis)
        state.a_theta_z[0] = ds[2];
        state.a_theta_z[1] = ds[3];

        // Accumulate RK4 contributions
        solver.solve(&state);
    } while (!complete);

    solver.end();

    // Copy integrated results back into the pendulum
    a1 = static_cast<float>(state.theta_z[0]);
    a2 = static_cast<float>(state.theta_z[1]);
    av1 = static_cast<float>(state.v_theta_z[0]);
    av2 = static_cast<float>(state.v_theta_z[1]);

    // Clean up allocated arrays in SystemState
    state.destroy();
}

std::array<double,4> DoublePendulum::toVector() const {
    return { static_cast<double>(a1), static_cast<double>(a2),
             static_cast<double>(av1), static_cast<double>(av2) };
}

void DoublePendulum::fromVector(const std::array<double,4>& v) {
    a1 = static_cast<float>(v[0]);
    a2 = static_cast<float>(v[1]);
    av1 = static_cast<float>(v[2]);
    av2 = static_cast<float>(v[3]);
}

void DoublePendulum::computeDerivatives(const std::array<double,4>& s, std::array<double,4>& ds) const {
    // s = {a1, a2, av1, av2}
    double A1 = s[0];
    double A2 = s[1];
    double AV1 = s[2];
    double AV2 = s[3];

    // derivatives of angles are angular velocities
    ds[0] = AV1;
    ds[1] = AV2;

    // compute angular accelerations using same formulas but with values from s
    double num1 = -g * (2 * m1 + m2) * sin(A1)
                  - m2 * g * sin(A1 - 2 * A2)
                  - 2 * sin(A1 - A2) * m2 * (AV2 * AV2 * l2 + AV1 * AV1 * l1 * cos(A1 - A2));
    double den1 = l1 * (2 * m1 + m2 - m2 * cos(2 * A1 - 2 * A2));
    ds[2] = num1 / den1;

    double num2 = 2 * sin(A1 - A2) *
                  (AV1 * AV1 * l1 * (m1 + m2) + g * (m1 + m2) * cos(A1) + AV2 * AV2 * l2 * m2 * cos(A1 - A2));
    double den2 = l2 * (2 * m1 + m2 - m2 * cos(2 * A1 - 2 * A2));
    ds[3] = num2 / den2;
}

Vector3 DoublePendulum::getPos1(Vector3 origin) const {
    return {(float)(origin.x + l1 * sin(a1)), (float)(origin.y - l1 * cos(a1)), (float)(0.0f)};
}

Vector3 DoublePendulum::getPos2(Vector3 origin) const {
    Vector3 pos1 = getPos1(origin);
    return {(float)(pos1.x + l2 * sin(a2)), (float)(pos1.y - l2 * cos(a2)), (float)(0.0f)};
}