//
// Created by Daniel Schatz on 04.11.25.
//

#ifndef PLUSSIM_RK4SOLVER_H
#define PLUSSIM_RK4SOLVER_H


#include "../self/solver.h"


class Rk4Solver : public Solver {
public:
    enum class RkStage {
        Stage_1,
        Stage_2,
        Stage_3,
        Stage_4,
        Complete,
        Undefined
    };

public:
    Rk4Solver();
    virtual ~Rk4Solver();

    virtual void start(SystemState *initial, double dt);
    virtual bool step(SystemState *system);
    virtual void solve(SystemState *system);
    virtual void end();

protected:
    static RkStage getNextStage(RkStage stage);

protected:
    RkStage m_stage;
    RkStage m_nextStage;

    SystemState m_initialState;
    SystemState m_accumulator;
};


#endif //PLUSSIM_RK4SOLVER_H