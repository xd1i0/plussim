//
// Created by Daniel Schatz on 02.11.25.
//

#ifndef PLUSSIM_DOUBLE_PENDULUM_H
#define PLUSSIM_DOUBLE_PENDULUM_H

#include "raylib.h"

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
    Vector2 getPos1(Vector2 origin) const;
    Vector2 getPos2(Vector2 origin) const;
    float getMass1() const { return m1; }
    float getMass2() const { return m2; }
};


#endif //PLUSSIM_DOUBLE_PENDULUM_H