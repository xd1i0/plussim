//
// Created by Daniel Schatz on 25.11.26.
//

#include "gtest/gtest.h"
#include "stellaris/gravity.h"
#include "stellaris/rk4Solver.h"
#include "stellaris/spring.h"
#include "stellaris/world.h"
#include <cmath>
#include <memory>

TEST(World, HangingSpringMatchesExactSolution) {
    const double mass = 1.0, g = 9.81, k = 50.0, rest = 1.0;
    const double A = 0.5, dt = 0.001;
    const int steps = 2000;

    const double y_eq = -(rest + mass * g / k);

    World world(std::make_unique<Rk4Solver>());

    BodyDef def;
    def.mass = mass;
    def.y = y_eq - A;
    const int body = world.addBody(def);

    world.addForce(std::make_unique<Gravity>(g));
    world.addForce(std::make_unique<Spring>(body, 0.0, 0.0, 0.0, k, 0.0, rest));

    

    for (int i = 0; i < steps; ++i) {
        world.step(dt);
    }

    const double t = steps * dt;
    const double expected = y_eq - A * std::cos(std::sqrt(k / mass) * t);
    EXPECT_NEAR(world.state().p_y[body], expected, 1e-6);
}