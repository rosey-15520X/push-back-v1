#pragma once

#include "main.h"
#include "state.h"

using namespace pros;

class PneumaticsHandler {
private:
    adi::DigitalOut& blockPiston;      // ScorerState.block
    adi::DigitalOut& middleGoalPiston; // IntakeState.middleGoal
    adi::DigitalOut& loaderPiston;     // LoaderState.contracted

    ScorerState& scorerState;
    IntakeState& intakeState;
    LoaderState& loaderState;

public:
    PneumaticsHandler(
        adi::DigitalOut& block, 
        adi::DigitalOut& middleGoal, 
        adi::DigitalOut& loader,
        ScorerState& scorer,
        IntakeState& intake,
        LoaderState& loaderSt
    ) : blockPiston(block), 
        middleGoalPiston(middleGoal), 
        loaderPiston(loader),
        scorerState(scorer),
        intakeState(intake),
        loaderState(loaderSt) {}

    void toggleBlock() {
        scorerState.block = !scorerState.block;
        blockPiston.set_value(scorerState.block);
    }

    void toggleMiddleGoal() {
        intakeState.middleGoal = !intakeState.middleGoal;
        middleGoalPiston.set_value(intakeState.middleGoal);
    }

    void toggleLoader() {
        loaderState.contracted = !loaderState.contracted;
        loaderPiston.set_value(loaderState.contracted);
    }

    void setBlock(bool extended) {
        scorerState.block = extended;
        blockPiston.set_value(extended);
    }

    void setMiddleGoal(bool extended) {
        intakeState.middleGoal = extended;
        middleGoalPiston.set_value(extended);
    }

    void setLoader(bool contracted) {
        loaderState.contracted = contracted;
        loaderPiston.set_value(contracted);
    }

    // Call this in init to ensure pistons match state
    void init() {
        blockPiston.set_value(scorerState.block);
        middleGoalPiston.set_value(intakeState.middleGoal);
        loaderPiston.set_value(loaderState.contracted);
    }
};
