//
// Created by Daniel Schatz on 07.11.25.
//

#ifndef PLUSSIM_EULERSOLVER_H
#define PLUSSIM_EULERSOLVER_H
#include "../self/solver.h"


class EulerSolver : public Solver {
public:
    EulerSolver();
    virtual ~EulerSolver();

    virtual void start(SystemState *initial, double dt);
    virtual bool step(SystemState *system);
    virtual void solve(SystemState *system);
    virtual void end();
};


#endif //PLUSSIM_EULERSOLVER_H