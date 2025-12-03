#pragma once

#include "main.h"
#include "state.h"

using namespace pros;

class ScorerHandler {
private:
    Motor& liftMotor;
    ScorerState& state;

public:
    ScorerHandler(Motor& lift, ScorerState& scorerState)
        : liftMotor(lift), state(scorerState) {}

    void toggle() {
        state.liftOn = !state.liftOn;
    }

    void toggleDirection() {
        state.liftDirection = !state.liftDirection;
    }

    void setOn(bool on) { state.liftOn = on; }
    void setDirection(bool up) { state.liftDirection = up; }

    void update() {
        if (state.liftOn) {
            int speed = state.liftDirection ? state.liftSpeed : -state.liftSpeed;
            liftMotor.move(speed);
        } else {
            liftMotor.move(0);
        }
    }

    const ScorerState& getState() const { return state; }
};

