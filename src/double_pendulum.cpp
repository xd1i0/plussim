//
// Created by Daniel Schatz on 02.11.25.
//

#include "double_pendulum.h"
#include <cmath>

DoublePendulum::DoublePendulum(float length1, float length2, float mass1, float mass2,
                               float angle1, float angle2, float timeStep)
    : l1(length1), l2(length2), m1(mass1), m2(mass2),
      a1(angle1), a2(angle2), dt(timeStep),
      av1(0.0f), av2(0.0f), aa1(0.0f), aa2(0.0f), g(9.81) {
}

void DoublePendulum::update() {
    // Physics calculations from myphysicslab.com
    aa1 = (-g * (2 * m1 + m2) * sin(a1) - m2 * g * sin(a1 - 2 * a2) -
           2 * sin(a1 - a2) * m2 * (av2 * av2 * l2 + av1 * av1 * l1 * cos(a1 - a2))) /
           (l1 * (2 * m1 + m2 - m2 * cos(2 * a1 - 2 * a2)));

    aa2 = (2 * sin(a1 - a2) * (av1 * av1 * l1 * (m1 + m2) + g * (m1 + m2) * cos(a1) +
           av2 * av2 * l2 * m2 * cos(a1 - a2))) /
           (l2 * (2 * m1 + m2 - m2 * cos(2 * a1 - 2 * a2)));

    av1 += aa1 * dt;
    av2 += aa2 * dt;
    a1 += av1 * dt;
    a2 += av2 * dt;
}

Vector2 DoublePendulum::getPos1(Vector2 origin) const {
    return {origin.x + l1 * sin(a1), origin.y + l1 * cos(a1)};
}

Vector2 DoublePendulum::getPos2(Vector2 origin) const {
    Vector2 pos1 = getPos1(origin);
    return {pos1.x + l2 * sin(a2), pos1.y + l2 * cos(a2)};
}