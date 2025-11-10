//
// Created by Daniel Schatz on 04.11.25.
//

#ifndef PLUSSIM_SOLVER_H
#define PLUSSIM_SOLVER_H

#include "system_state.h"

class Solver {
public:
    Solver();
    virtual ~Solver();

    virtual void start(SystemState *initial, double dt);
    virtual bool step(SystemState *system);
    virtual void solve(SystemState *system);
    virtual void end();

protected:
    double m_dt;
};


#endif //PLUSSIM_SOLVER_H