#pragma once

#include "main.h"
#include "state.h"

using namespace pros;

class PneumaticsHandler {
private:
    adi::DigitalOut& blockPiston;         // ScorerState.block
    adi::DigitalOut& middleGoalPiston;    // IntakeState.middleGoal
    adi::DigitalOut& loaderPiston;        // LoaderState.contracted
    adi::DigitalOut& armPiston;           // ArmState.extended

    ScorerState& scorerState;
    IntakeState& intakeState;
    LoaderState& loaderState;
    ArmState& armState;

public:
    PneumaticsHandler(
        adi::DigitalOut& block,
        adi::DigitalOut& middleGoal,
        adi::DigitalOut& loader,
        adi::DigitalOut& arm,
        ScorerState& scorer,
        IntakeState& intake,
        LoaderState& loaderSt,
        ArmState& armSt
    ) : blockPiston(block),
        middleGoalPiston(middleGoal),
        loaderPiston(loader),
        armPiston(arm),
        scorerState(scorer),
        intakeState(intake),
        loaderState(loaderSt),
        armState(armSt) {}

    // --- Toggles ---

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

    void toggleArm() {
        armState.extended = !armState.extended;
        armPiston.set_value(armState.extended);
    }


    // --- Setters ---

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

    void setArm(bool extended) {
        armState.extended = extended;
        armPiston.set_value(extended);
    }


    // --- Init sync ---

    void init() {
        blockPiston.set_value(scorerState.block);
        middleGoalPiston.set_value(intakeState.middleGoal);
        loaderPiston.set_value(loaderState.contracted);
        armPiston.set_value(armState.extended);
    }
};
