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
    chassis.setPose(-62.273, m * 17.703, mirror ? 180 : 0);

    followPath(PeriodMoveLoadLG1PT1_txt, 15, 3000, true, mirror);
    chassis.turnToHeading(mirror ? mirrorHeading(270) : 270, 1000);
    followPath(PeriodMoveLoadLG1PT2_txt, 15, 3000, true, mirror);
    followPath(PeriodMoveLoadLG1PT3_txt, 15, 3000, false, mirror);
    
    if (goals >= 2) {
        followPath(PeriodMoveLoadMGPT1_txt, 15, 3000, true, mirror);
        chassis.turnToHeading(mirror ? mirrorHeading(45) : 45, 1000);
        // RUN THE INTAKE
        followPath(PeriodMoveLoadMGPT2_txt, 15, 3000, true, mirror);
        chassis.turnToHeading(mirror ? mirrorHeading(225) : 225, 1000);
        followPath(PeriodMoveLoadMGPT3_txt, 15, 3000, false, mirror);
        
        if (goals >= 3) {
            followPath(PeriodMoveLoadMGPT4_txt, 15, 3000, true, mirror);
            chassis.turnToHeading(mirror ? mirrorHeading(270) : 270, 1000);
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
