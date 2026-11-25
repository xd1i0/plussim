//
// Created by Daniel Schatz on 07.11.25.
//

#ifndef PLUSSIM_CUBE_H
#define PLUSSIM_CUBE_H

#include "stellaris/spring.h"
#include "stellaris/world.h"

class Cube {
public:
    Cube(double mass, double x, double y, double z, double size);
    Cube(const Cube &) = delete;
    Cube &operator=(const Cube &) = delete;

    void update(double dt);
    void reset(double x, double y, double z);
    void getPosition(double &x, double &y, double &z) const;
    void getSpringAnchor(double &x, double &y, double &z) const;
    double getSize() const { return m_size; }

    void setSpring(double anchor_x, double anchor_y, double anchor_z, double spring_k, double damping, double rest_length);
    bool hasSpring() const { return m_spring != nullptr; }

    void setMass(double mass);
    void setSize(double size);
    void setSolverType(int type); // 0 = Euler, 1 = RK4

private:
    World m_world;
    int m_body;
    Spring *m_spring = nullptr; // owned by m_world

    double m_size;
    double m_initial_x, m_initial_y, m_initial_z;

    double m_spring_anchor_x, m_spring_anchor_y, m_spring_anchor_z;

    static constexpr double GRAVITY = 9.81;
};

#endif //PLUSSIM_CUBE_H

