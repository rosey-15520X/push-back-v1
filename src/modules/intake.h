#pragma once

#include "main.h"
#include "state.h"

using namespace pros;

class IntakeHandler {
private:
    Motor& primaryMotor;
    Motor& secondaryMotor;
    IntakeState& state;

public:
    IntakeHandler(Motor& primary, Motor& secondary, IntakeState& intakeState)
        : primaryMotor(primary), secondaryMotor(secondary), state(intakeState) {}

    void toggle() {
        state.primaryOn = !state.primaryOn;
        state.secondaryOn = !state.secondaryOn;
    }

    void toggleDirection() {
        state.primaryDirection = !state.primaryDirection;
        state.secondaryDirection = !state.secondaryDirection;
    }

    void setPrimaryOn(bool on) { state.primaryOn = on; }
    void setSecondaryOn(bool on) { state.secondaryOn = on; }
    void setPrimaryDirection(bool forward) { state.primaryDirection = forward; }
    void setSecondaryDirection(bool forward) { state.secondaryDirection = forward; }

    void update() {
        // Handle primary motor
        if (state.primaryOn) {
            int speed = state.primaryDirection ? state.primarySpeed : -state.primarySpeed;
            primaryMotor.move(speed);
        } else {
            primaryMotor.move(0);
        }

        // Handle secondary motor
        if (state.secondaryOn) {
            int speed = state.secondaryDirection ? state.secondarySpeed : -state.secondarySpeed;
            secondaryMotor.move(speed);
        } else {
            secondaryMotor.move(0);
        }
    }

    const IntakeState& getState() const { return state; }
};

