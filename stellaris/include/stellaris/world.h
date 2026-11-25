//
// Created by Daniel Schatz on 25.11.26.
//

#ifndef PLUSSIM_WORLD_H
#define PLUSSIM_WORLD_H
#include <memory>
#include <utility>
#include <vector>

#include "stellaris/force_generator.h"
#include "stellaris/solver.h"
#include "stellaris/system_state.h"

struct BodyDef {
    double mass = 1.0;
    double x = 0.0, y = 0.0, z = 0.0;
    double v_x = 0.0, v_y = 0.0, v_z = 0.0;
};

class World {
public:
    explicit World(std::unique_ptr<Solver> solver) : m_solver(std::move(solver)) {}

    int addBody(const BodyDef &body);
    void addForce(std::unique_ptr<ForceGenerator> force) {
        m_forces.push_back(std::move(force));
    }
    void step(double dt);
    void setSolverType(std::unique_ptr<Solver> solver) {
        m_solver = std::move(solver);
    }

    SystemState &state() { return m_state; }
    const SystemState &state() const { return m_state; }

private:
    SystemState m_state;
    std::unique_ptr<Solver> m_solver;
    std::vector<std::unique_ptr<ForceGenerator>> m_forces;

    void computeAccelerations();
};
#endif //PLUSSIM_WORLD_H
