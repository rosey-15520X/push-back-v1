#pragma once

#include "main.h"
#include "state.h"

using namespace pros;

class PneumaticsHandler {
private:
    adi::DigitalOut& blockPiston;         // ScorerState.block
    adi::DigitalOut& loaderPiston;        // LoaderState.contracted
    adi::DigitalOut& armPiston;           // ArmState.extended
    adi::DigitalOut& middleGoalPiston;           // ArmState.extended


    ScorerState& scorerState;
    IntakeState& intakeState;
    LoaderState& loaderState;
    ArmState& armState;

public:
    PneumaticsHandler(
        adi::DigitalOut& block,
        adi::DigitalOut& loader,
        adi::DigitalOut& middleGoal,
        adi::DigitalOut& arm,
        ScorerState& scorer,
        IntakeState& intake,
        LoaderState& loaderSt,
        ArmState& armSt
    ) : blockPiston(block),
        loaderPiston(loader),
        armPiston(arm),
        scorerState(scorer),
        intakeState(intake),
        loaderState(loaderSt),
        armState(armSt),
        middleGoalPiston(middleGoal) {}

    // --- Getters ---
    bool getMiddleGoal() {
        return scorerState.middleGoal;
    }

    void toggleBlock() {
        scorerState.block = !scorerState.block;
        blockPiston.set_value(scorerState.block);
    }

    void toggleLoader() {
        loaderState.contracted = !loaderState.contracted;
        loaderPiston.set_value(loaderState.contracted);
    }

    void toggleMiddleGoal() {
        scorerState.middleGoal = !scorerState.middleGoal;
        middleGoalPiston.set_value(scorerState.middleGoal);
    }

    void toggleArm() {
        armState.extended = !armState.extended;
        armPiston.set_value(armState.extended);
    }


    // --- Setters ---

    void setBlock(bool extended) {
        scorerState.block = extended;
        blockPiston.set_value(extended);
    }

    void setLoader(bool contracted) {
        loaderState.contracted = contracted;
        loaderPiston.set_value(contracted);
    }

    void setMiddleGoal(bool extended) {
        middleGoalPiston.set_value(extended);
    }

    void setArm(bool extended) {
        armState.extended = extended;
        armPiston.set_value(extended);
    }


    // --- Init sync ---

    void init() {
        blockPiston.set_value(scorerState.block);
        loaderPiston.set_value(loaderState.contracted);
        armPiston.set_value(armState.extended);
    }
};
