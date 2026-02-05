#pragma once

#include "main.h"
#include "lemlib/asset.hpp"
#include "auton_helpers.h"
#include "../modules/drive.h"

// Path assets for Period auton
ASSET(PeriodMoveLoadLG1PT1_txt)
ASSET(PeriodMoveLoadLG1PT2_txt)
ASSET(PeriodMoveLoadLG1PT3_txt)
ASSET(PeriodMoveLoadMGPT1_txt)
ASSET(PeriodMoveLoadMGPT2_txt)
ASSET(PeriodMoveLoadMGPT3_txt)
ASSET(PeriodMoveLoadMGPT4_txt)
ASSET(PeriodMoveLoadMGPT5_txt)
ASSET(PeriodMoveLoadMGPT6_txt)
ASSET(PeriodMoveLoadLG1PT4_txt)

/**
 * Helper to follow a path with optional mirroring
 * Uses asset struct for the path data
 */
inline void followPath(asset pathAsset, float lookahead, int timeout, bool forwards, bool mirror) {
    if (mirror) {
        const char* mirrored = mirrorPath((const char*)pathAsset.buf);
        asset mirroredAsset = {(uint8_t*)mirrored, strlen(mirrored)};
        chassis.follow(mirroredAsset, lookahead, timeout, forwards);
    } else {
        chassis.follow(pathAsset, lookahead, timeout, forwards);
    }
    chassis.waitUntilDone();  // IMPORTANT: Wait for path to complete before continuing!
}

/**
 * Autonomous routine for Period - works for both sides!
 * @param intake - IntakeHandler reference
 * @param goals - Number of goals to score (1, 2, or 3)
 * @param mirror - If true, mirrors for right side
 */
inline void autonPeriod(IntakeHandler& intake, adi::DigitalOut& loaderPiston, int goals, bool mirror = false) {
    float m = mirror ? -1.0f : 1.0f;  // Y multiplier
    
    // Starting position (mirrored Y if on right side)
    chassis.setPose(-54.996, 17.593, mirror ? 180 : 0);

    followPath(PeriodMoveLoadLG1PT1_txt, 15, 3000, true, mirror);
    chassis.turnToHeading(mirror ? mirrorHeading(270) : 270, 1000);
    chassis.waitUntilDone();
    followPath(PeriodMoveLoadLG1PT2_txt, 15, 3000, true, mirror);
    followPath(PeriodMoveLoadLG1PT3_txt, 15, 3000, false, mirror);
    // Recalibrate position (heading 270, or 90 if mirrored)
    // chassis.setPose(-30.66, 47.244 * (mirror ? -1 : 1), mirror ? mirrorHeading(270) : 270);
    // // // Move forward 17 inches to target
    // chassis.moveToPoint(-47.329, 47.329 * (mirror ? -1 : 1), 2000, {.forwards = true});
    // chassis.waitUntilDone();
    followPath(PeriodMoveLoadLG1PT4_txt, 15, 3000, true, mirror);
    chassis.turnToHeading(mirror ? mirrorHeading(135) : 135, 1000);
    chassis.waitUntilDone();
    if (goals >= 2) {
        // Recalibrate odometry - PT4 ends here, we turned to 135
        chassis.setPose(-47.932, 47.932 * (mirror ? -1 : 1), mirror ? mirrorHeading(135) : 135);
        // Path starts at (-47.932, 47.932) heading 135
        followPath(PeriodMoveLoadMGPT1_txt, 12, 3000, true, mirror);  // Reduced lookahead for accuracy
        // RUN THE INTAKE
        // followPath(PeriodMoveLoadMGPT2_txt, 15, 3000, true, mirror);
        chassis.turnToHeading(mirror ? mirrorHeading(315) : 315, 1000);
        chassis.waitUntilDone();
        followPath(PeriodMoveLoadMGPT3_txt, 15, 3000, false, mirror);
        
        if (goals >= 3) {
            followPath(PeriodMoveLoadMGPT4_txt, 15, 3000, true, mirror);
            chassis.turnToHeading(mirror ? mirrorHeading(270) : 270, 1000);
            chassis.waitUntilDone();
            followPath(PeriodMoveLoadMGPT5_txt, 15, 1500, true, mirror);
            followPath(PeriodMoveLoadMGPT6_txt, 15, 1500, false, mirror);
        }
    }
}

// Convenience wrappers
inline void autonPeriodLeft(IntakeHandler& intake, adi::DigitalOut& loaderPiston, int goals) {
    autonPeriod(intake, loaderPiston, goals, false);
}

inline void autonPeriodRight(IntakeHandler& intake, adi::DigitalOut& loaderPiston, int goals) {
    autonPeriod(intake, loaderPiston, goals, true);
}
