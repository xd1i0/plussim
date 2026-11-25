//
// Created by Daniel Schatz on 25.11.26.
//

#ifndef PLUSSIM_GRAVITY_H
#define PLUSSIM_GRAVITY_H
#include "stellaris/force_generator.h"

class Gravity : public ForceGenerator {
public:
    explicit Gravity(double g) : m_g(g) {}
    void apply(SystemState &state) override;
private:
    double m_g;
};
#endif //PLUSSIM_GRAVITY_H
