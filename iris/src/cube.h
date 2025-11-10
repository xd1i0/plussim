//
// Created by Daniel Schatz on 07.11.25.
//

#ifndef PLUSSIM_CUBE_H
#define PLUSSIM_CUBE_H

#include "../external/self/system_state.h"
#include "../external/self/eulerSolver.h"
#include "../external/self/rk4Solver.h"

class Cube {
public:
    Cube(double mass, double x, double y, double z, double size);
    ~Cube();

    void update(double dt);
    void reset(double x, double y, double z);
    void getPosition(double &x, double &y, double &z) const;
    void getSpringAnchor(double &x, double &y, double &z) const;
    double getSize() const { return m_size; }

    void setSpring(double anchor_x, double anchor_y, double anchor_z, double spring_k, double damping, double rest_length);
    bool hasSpring() const { return m_spring_enabled; }

    void setMass(double mass);
    void setSize(double size);
    void setSolverType(int type); // 0 = Euler, 1 = RK4

private:
    SystemState m_state;
    Solver* m_solver;
    EulerSolver m_euler_solver;
    Rk4Solver m_rk4_solver;
    int m_solver_type;

    double m_size;
    double m_initial_x, m_initial_y, m_initial_z;

    // Spring properties
    bool m_spring_enabled;
    double m_spring_anchor_x, m_spring_anchor_y, m_spring_anchor_z;
    double m_spring_k;        // Spring constant
    double m_spring_damping;  // Damping coefficient
    double m_spring_rest_length;

    static constexpr double GRAVITY = 9.81;
};

#endif //PLUSSIM_CUBE_H

