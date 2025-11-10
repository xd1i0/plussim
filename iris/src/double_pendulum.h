//
// Created by Daniel Schatz on 02.11.25.
//

#ifndef PLUSSIM_DOUBLE_PENDULUM_H
#define PLUSSIM_DOUBLE_PENDULUM_H

#include "raylib.h"
#include <array>
#include <vector>

class DoublePendulum {
private:
    double g;           // Acceleration due to gravity
    float l1, l2;       // Lengths of pendulums
    float m1, m2;       // Masses
    float a1, a2;       // Angles
    float av1, av2;     // Angular velocities
    float aa1, aa2;     // Angular accelerations
    float dt;           // Time step

public:
    DoublePendulum(float length1, float length2, float mass1, float mass2,
                   float angle1, float angle2, float timeStep = 0.05f);

    void update();
    Vector3 getPos1(Vector3 origin) const;
    Vector3 getPos2(Vector3 origin) const;
    float getMass1() const { return m1; }
    float getMass2() const { return m2; }

    // Adapter for external solvers
    std::array<double,4> toVector() const;                       // {a1, a2, av1, av2}
    void fromVector(const std::array<double,4>& v);              // apply integrated result
    void computeDerivatives(const std::array<double,4>& s, std::array<double,4>& ds) const;
};


#endif //PLUSSIM_DOUBLE_PENDULUM_H