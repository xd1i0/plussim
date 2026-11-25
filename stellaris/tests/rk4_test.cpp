#include "gtest/gtest.h"
#include "stellaris/rk4Solver.h"
#include <cmath>

namespace {

    // x'' = -x, x(0) = 1, v(0) = 0  =>  x(t) = cos(t)
    double oscillatorError(int steps) {
        SystemState state;
        state.resize(1, 0);

        for (double *a : {state.a_theta_x, state.a_theta_y, state.a_theta_z,
                          state.v_theta_x, state.v_theta_y, state.v_theta_z,
                          state.theta_x, state.theta_y, state.theta_z,
                          state.a_x, state.a_y, state.a_z,
                          state.v_x, state.v_y, state.v_z,
                          state.p_x, state.p_y, state.p_z,
                          state.f_x, state.f_y, state.f_z,
                          state.t_x, state.t_y, state.t_z}) {
            a[0] = 0.0;
                          }
        state.m[0] = 1.0;
        state.p_x[0] = 1.0;

        const double duration = 2.0;
        Rk4Solver solver;

        for (int i = 0; i < steps; ++i) {
            solver.start(&state, duration / steps);
            bool complete = false;
            do {
                complete = solver.step(&state);
                state.a_x[0] = -state.p_x[0];
                solver.solve(&state);
            } while (!complete);
            solver.end();
        }

        const double error = std::abs(state.p_x[0] - std::cos(duration));
        state.destroy();
        return error;
    }

}  // namespace

TEST(Rk4Solver, IsFourthOrder) {
    const double ratio = oscillatorError(20) / oscillatorError(40);
    EXPECT_NEAR(ratio, 16.0, 1.0);
}
