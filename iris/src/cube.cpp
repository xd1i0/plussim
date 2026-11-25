#include "cube.h"

#include <memory>
#include <utility>

#include "stellaris/eulerSolver.h"
#include "stellaris/gravity.h"
#include "stellaris/rk4Solver.h"

Cube::Cube(double mass, double x, double y, double z, double size)
    : m_world(std::make_unique<EulerSolver>()),
      m_size(size), m_initial_x(x), m_initial_y(y), m_initial_z(z),
      m_spring_anchor_x(0.0), m_spring_anchor_y(0.0), m_spring_anchor_z(0.0)
{
    BodyDef body;
    body.mass = mass;
    body.x = x;
    body.y = y;
    body.z = z;
    m_body = m_world.addBody(body);

    m_world.addForce(std::make_unique<Gravity>(GRAVITY));
}

void Cube::update(double dt) {
    m_world.step(dt);

    SystemState &state = m_world.state();
    if (state.p_y[m_body] <= m_size / 2.0) {
        state.p_y[m_body] = m_size / 2.0;
        if (state.v_y[m_body] < 0.0) {
            state.v_y[m_body] = 0.0;
        }
    }
}

void Cube::reset(double x, double y, double z) {
    m_initial_x = x;
    m_initial_y = y;
    m_initial_z = z;

    SystemState &state = m_world.state();
    state.p_x[m_body] = x;
    state.p_y[m_body] = y;
    state.p_z[m_body] = z;
    state.v_x[m_body] = state.v_y[m_body] = state.v_z[m_body] = 0.0;
}

void Cube::getPosition(double &x, double &y, double &z) const {
    const SystemState &state = m_world.state();
    x = state.p_x[m_body];
    y = state.p_y[m_body];
    z = state.p_z[m_body];
}

void Cube::setSpring(double anchor_x, double anchor_y, double anchor_z,
                     double spring_k, double damping, double rest_length) {
    m_spring_anchor_x = anchor_x;
    m_spring_anchor_y = anchor_y;
    m_spring_anchor_z = anchor_z;

    if (m_spring) {
        m_spring->set(anchor_x, anchor_y, anchor_z, spring_k, damping, rest_length);
        return;
    }

    auto spring = std::make_unique<Spring>(m_body, anchor_x, anchor_y, anchor_z,
                                           spring_k, damping, rest_length);
    m_spring = spring.get();
    m_world.addForce(std::move(spring));
}

void Cube::getSpringAnchor(double &x, double &y, double &z) const {
    x = m_spring_anchor_x;
    y = m_spring_anchor_y;
    z = m_spring_anchor_z;
}

void Cube::setMass(double mass) {
    m_world.state().m[m_body] = mass;
}

void Cube::setSize(double size) {
    m_size = size;
}

void Cube::setSolverType(int type) {
    if (type == 0) {
        m_world.setSolverType(std::make_unique<EulerSolver>());
    } else if (type == 1) {
        m_world.setSolverType(std::make_unique<Rk4Solver>());
    }
}
