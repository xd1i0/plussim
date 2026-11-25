//
// Created by Daniel Schatz on 25.11.26.
//

#ifndef PLUSSIM_FORCE_GENERATOR_H
#define PLUSSIM_FORCE_GENERATOR_H
#include "stellaris/system_state.h"

class ForceGenerator {
public:
    virtual ~ForceGenerator() = default;
    virtual void apply(SystemState &state) = 0;
};

#endif //PLUSSIM_FORCE_GENERATOR_H
