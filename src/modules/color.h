#pragma once
#include "intake.h"
#include "scorer.h"
#include "pneumatics.h"

inline bool isRed(int hue) {
    return hue < 30 || hue > 330;
}

inline bool isBlue(int hue) {
    return hue < 250 && hue > 170;
}

inline bool isBall(int hue) {
    return isRed(hue) || isBlue(hue);
}


inline bool isOpponentBall(int hue, bool isRedAlliance) {
    // Fixed: proper parentheses for operator precedence
    return isBall(hue) && ((isRed(hue) && !isRedAlliance) || (isBlue(hue) && isRedAlliance));
}

inline void handleColor(int hueUpper, int hueBottom, IntakeHandler& intake, ScorerHandler& scorer, PneumaticsHandler& pneumatics, bool isRedAlliance, int& lastDetectedTop, bool scoringMiddleGoal) {
    // Track when ball first detected at top sensor
    if (isBall(hueUpper)) {
        if (lastDetectedTop == 0) {
            lastDetectedTop = pros::millis();  // Record first detection time
        }
        // Keep timestamp while ball is present
    } else {
        lastDetectedTop = 0;  // Reset when ball leaves
    }

    if (isBall(hueUpper) && isBall(hueBottom) && !scorer.getState().block) { // issue u wanna score and u turn on scorer before releasing block
        if (isOpponentBall(hueUpper, isRedAlliance)) {
            // quickly eject
            pneumatics.setBlock(true);
            bool tp_direction = scorer.getState().liftDirection;
            scorer.setOn(true);

            scorer.setDirection(false);
            
            delay(200);
            scorer.setOn(false);
            scorer.setDirection(tp_direction);  

            pneumatics.setBlock(false);
        } 
        else {
            // Check if ball has been at sensor for more than 1500ms
            if (lastDetectedTop > 0 && (pros::millis() - lastDetectedTop > 1500) && !scoringMiddleGoal) {
                scorer.setOn(false);
            }
        }
    }

    if (isBall(hueBottom) && scorer.getState().liftOn && scorer.getState().block) {
        if (isOpponentBall(hueBottom, isRedAlliance) || isOpponentBall(hueUpper, isRedAlliance)) {
            bool direction = scorer.getState().liftDirection;
            pneumatics.setBlock(true);
            intake.setSecondarySpeed(60);
            intake.update();
            scorer.setDirection(!direction);
            scorer.setLiftSpeed(20);
            scorer.update();
            delay(50);
            scorer.setDirection(direction);
            scorer.setLiftSpeed(127);
            scorer.update();
            intake.setSecondarySpeed(127);
            intake.update();
        }
    }
}

// bool isRed = hue < 30 || hue > 330;
// bool isBlue = hue < 250 && hue > 170;