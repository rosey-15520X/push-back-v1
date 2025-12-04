#pragma once

#define INTAKE_BASE_PRIMARY_SPEED 127
#define INTAKE_BASE_SECONDARY_SPEED 127

#define SCORER_SPEED 127

struct IntakeState {
    // Intake base
    bool primaryDirection = true;
    bool primaryOn = false;
    int primarySpeed = INTAKE_BASE_PRIMARY_SPEED;

    // Intake top
    bool secondaryDirection = true;
    bool secondaryOn = false;
    int secondarySpeed = INTAKE_BASE_SECONDARY_SPEED;

    bool middleGoal = false;
};

struct ScorerState {
    bool liftDirection = true;
    bool liftOn = false;
    int liftSpeed = SCORER_SPEED;

    bool block = false;
};

struct LoaderState {
    bool contracted = false;
};

struct ArmState {
    bool extended = false; // default = contracted
};

struct State {
    bool driveDirection = true;
    IntakeState intake;
    ScorerState scorer;
    LoaderState loader;
    ArmState arm; 
};