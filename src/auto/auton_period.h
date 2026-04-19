#pragma once

#include "main.h"
#include "lemlib/asset.hpp"
#include "auton_helpers.h"
#include "../modules/drive.h"


// Path assets for Period auton
ASSET(PeriodMoveLoadLG1PT1_txt)
ASSET(PeriodMoveLoadLG1PT2_txt)
ASSET(PeriodMoveLoadLG1PT3_txt)
ASSET(ELIMPeriodMoveLoadLG1PT1_txt)
ASSET(ELIMPeriodMoveLoadLG1PT2_txt)
ASSET(ELIMPeriodMoveLoadLG1PT3_txt)
ASSET(ELIMPeriodMoveLoadLG1PT4_txt)
ASSET(ELIMPeriodMoveLoadLG1PT05_txt)
ASSET(ELIMPeriodMoveLoadLG1PT06_txt)
ASSET(RELIMPeriodMoveLoadLG1PT1_txt)
ASSET(RELIMPeriodMoveLoadLG1PT2_txt)
ASSET(RELIMPeriodMoveLoadLG1PT3_txt)
ASSET(RELIMPeriodMoveLoadLG1PT4_txt)
ASSET(RELIMPeriodMoveLoadLG1PT05_txt)
ASSET(RELIMPeriodMoveLoadLG1PT06_txt)
ASSET(RIGHTPeriodMoveLoadLG1PT1_txt)
ASSET(RIGHTPeriodMoveLoadLG1PT2_txt)
ASSET(RIGHTPeriodMoveLoadLG1PT3_txt)
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
    chassis.waitUntilDone();
}

/**
 * Autonomous routine for Period - works for both sides!
 * @param intake - IntakeHandler reference
 * @param goals - Number of goals to score (1, 2, or 3)
 * @param mirror - If true, mirrors for right side
 */
inline void autonPeriod(IntakeHandler& intake, ScorerHandler& scorer, adi::DigitalOut& loaderPiston,  adi::DigitalOut& scorerPiston, int goals, bool mirror = false) {
    float m = mirror ? -1.0f : 1.0f;  // Y multiplier
    
    // Starting position 
    chassis.setPose(-54.996, 17.593, mirror ? 180 : 0);

    // Move in front of match loader
    followPath(PeriodMoveLoadLG1PT1_txt, 12, 3000, true, mirror);
    loaderPiston.set_value(true);

    chassis.turnToHeading(mirror ? mirrorHeading(270) : 270, 1000);
    chassis.waitUntilDone();
    // deploy loader
    // retract scorer
    scorerPiston.set_value(false);
    // Go to match loader and load

    intake.setPrimaryOn(true);
    intake.setSecondaryOn(true);
    intake.setSecondaryDirection(true);  // forward
    intake.update();

    followPath(PeriodMoveLoadLG1PT2_txt, 12, 1500, true, mirror);

    chassis.setPose(-56.288, 45.537, 270);
    chassis.moveToPoint(-61.215, 46.209, 800, {.forwards = true});
    chassis.moveToPoint(-60.215, 46.209, 200, {.forwards = false});
    chassis.moveToPoint(-61.215, 46.209, 800, {.forwards = true});

    // chassis.moveToPoint(-59.215, 46.209, 500, {.forwards = false});
    // chassis.moveToPoint(-61.215, 46.209, 500, {.forwards = true});

    chassis.setPose(-61.215, 46.209, 270);

    pros::delay(400);

    intake.setPrimaryOn(false);
    intake.setSecondaryOn(false);
    intake.update();

    // // chassis.setPose(-56.512, 46.657, 270); // RESET POSITION
    // loadBalls(intake, 3000);
    // go TO high goal
    loaderPiston.set_value(false);
    // chassis.setPose(-30.084, 46.881, 270); // RESET POSITION

    // chassis.setPose(-30.66, 47.244, 270);
    scorerPiston.set_value(true);
    followPath(PeriodMoveLoadLG1PT3_txt, 12, 1800, false, mirror);
    intake.setPrimaryOn(true);
    intake.setSecondaryOn(true);
    intake.setSecondaryDirection(true);
    intake.setPrimaryDirection(true);
    scorer.setOn(true);
    scorer.setDirection(true);
    scorer.update();

    intake.update();

    delay(2000);
    // scoreBalls(intake, 3000, false, scorer);
    // Recalibrate position (heading 270, or 90 if mirrored)
    // chassis.setPose(-30.66, 47.244 * (mirror ? -1 : 1), mirror ? mirrorHeading(270) : 270);
    // // // Move forward 17 inches to target
    // chassis.moveToPoint(-47.329, 47.329 * (mirror ? -1 : 1), 2000, {.forwards = true});
    // chassis.waitUntilDone();
    scorerPiston.set_value(false);
    // Move away
    followPath(PeriodMoveLoadLG1PT4_txt, 15, 3000, true, mirror);
    chassis.setPose(-47.932, 47.932, 270);
    chassis.turnToHeading(mirror ? mirrorHeading(135) : 135, 1000);
    chassis.waitUntilDone();
    if (goals >= 2) {
        // Recalibrate odometry - PT4 ends here, we turned to 135
        // chassis.setPose(-47.932, 47.932 * (mirror ? -1 : 1), mirror ? mirrorHeading(135) : 135);
        // Path starts at (-47.932, 47.932) heading 135
        intake.setPrimaryOn(true);
        intake.setSecondaryOn(true);
        intake.update();
        followPath(PeriodMoveLoadMGPT1_txt, 12, 3000, true, mirror);  // Move to middle goal
        delay(500);
        // RUN THE INTAKE
        // followPath(PeriodMoveLoadMGPT2_txt, 15, 3000, true, mirror);
        chassis.turnToHeading(mirror ? mirrorHeading(320) : 320, 1000); // Turn to middle goal
        intake.setPrimaryOn(false);
        intake.setSecondaryOn(false);
        intake.setSecondaryDirection(true);
        intake.update();
        chassis.waitUntilDone();
        followPath(PeriodMoveLoadMGPT3_txt, 15, 1300, false, mirror); // line up with middle goal
        scorer.setDirection(false);
        scorer.setOn(true);
        scorer.update();

        scorerPiston.set_value(false);
        intake.setPrimaryOn(true);
        intake.setSecondaryOn(true);
        intake.setPrimaryDirection(false);
        intake.setSecondaryDirection(false);
        intake.update();
        delay(100);

                intake.setPrimaryOn(true);
        intake.setSecondaryOn(true);


        intake.setPrimaryDirection(true);
        intake.setSecondaryDirection(true);
        scorer.setDirection(false);
        scorer.setOn(true);
        scorer.update();
        intake.update();
        delay(3000);

        // scoreBalls(intake, 3000, true, scorer);
        scorerPiston.set_value(false);
        
        if (goals >= 3) {
            // Move to low goal
            followPath(PeriodMoveLoadMGPT4_txt, 15, 3000, true, mirror);
            chassis.turnToHeading(mirror ? mirrorHeading(270) : 270, 1000);
            chassis.waitUntilDone();
            intake.setPrimaryOn(true);
            intake.setSecondaryOn(true);
            intake.setSecondaryDirection(true);  // forward
            intake.update();
            loaderPiston.set_value(false);
            followPath(PeriodMoveLoadMGPT5_txt, 12, 1500, true, mirror);
            loadBalls(intake, 500);
            followPath(PeriodMoveLoadMGPT6_txt, 12, 1500, false, mirror);
            scorerPiston.set_value(true);
            scorer.setOn(true);
            scorer.setDirection(false);
            scorer.update();
            intake.update();
            delay(3000);
            intake.setPrimaryOn(false);
            intake.setSecondaryOn(false);
            scorer.setOn(false);
            scorer.update();
            intake.update();

        }
    }
}

// Convenience wrappers
inline void autonPeriodLeft(IntakeHandler& intake, ScorerHandler& scorer, adi::DigitalOut& loaderPiston, adi::DigitalOut& scorerPiston, int goals) {
    autonPeriod(intake, scorer, loaderPiston, scorerPiston, goals, false);
}

inline void autonPeriodRight(IntakeHandler& intake, ScorerHandler& scorer, adi::DigitalOut& loaderPiston, adi::DigitalOut& scorerPiston, int goals) {
    autonPeriod(intake, scorer, loaderPiston, scorerPiston, goals, true);
}


inline void autonPeriodRightOne(IntakeHandler& intake, ScorerHandler& scorer, adi::DigitalOut& loaderPiston,  adi::DigitalOut& scorerPiston, int goals, bool mirror = false) {
    float m = mirror ? -1.0f : 1.0f;  // Y multiplier
    
    // Starting position 
    chassis.setPose(-54.996, -17.593, 180);

    // Move in front of match loader
    followPath(RIGHTPeriodMoveLoadLG1PT1_txt, 12, 3000, true, mirror);
    loaderPiston.set_value(true);

    chassis.turnToHeading(mirror ? mirrorHeading(270) : 270, 1000);
    chassis.waitUntilDone();
    // deploy loader
    // retract scorer
    scorerPiston.set_value(false);
    // Go to match loader and load

    intake.setPrimaryOn(true);
    intake.setSecondaryOn(true);
    intake.setSecondaryDirection(true);  // forward
    intake.update();

    followPath(RIGHTPeriodMoveLoadLG1PT2_txt, 12, 1500, true, mirror);

    chassis.setPose(-56.288, -45.537, 270);
    chassis.moveToPoint(-61.215, -46.209, 800, {.forwards = true});
    chassis.moveToPoint(-60.215, -46.209, 200, {.forwards = false});
    chassis.moveToPoint(-61.215, -46.209, 800, {.forwards = true});

    // chassis.moveToPoint(-59.215, 46.209, 500, {.forwards = false});
    // chassis.moveToPoint(-61.215, 46.209, 500, {.forwards = true});

    chassis.setPose(-61.215, -46.209, 270);

    pros::delay(600);

    intake.setPrimaryOn(false);
    intake.setSecondaryOn(false);
    intake.update();

    // // chassis.setPose(-56.512, 46.657, 270); // RESET POSITION
    // loadBalls(intake, 3000);
    // go TO high goal
    loaderPiston.set_value(false);
    // chassis.setPose(-30.084, 46.881, 270); // RESET POSITION

    // chassis.setPose(-30.66, 47.244, 270);
    followPath(RIGHTPeriodMoveLoadLG1PT3_txt, 12, 1800, false, mirror);
    scorerPiston.set_value(true);

    intake.setPrimaryOn(true);
    intake.setSecondaryOn(true);
    intake.setSecondaryDirection(true);
    intake.setPrimaryDirection(true);
    scorer.setOn(true);
    scorer.setDirection(true);
    scorer.update();

    intake.update();

    delay(2000);
    // scoreBalls(intake, 3000, false, scorer);
    // Recalibrate position (heading 270, or 90 if mirrored)
    // chassis.setPose(-30.66, 47.244 * (mirror ? -1 : 1), mirror ? mirrorHeading(270) : 270);
    // // // Move forward 17 inches to target
    // chassis.moveToPoint(-47.329, 47.329 * (mirror ? -1 : 1), 2000, {.forwards = true});
    // chassis.waitUntilDone();
    scorerPiston.set_value(false);
    // back up
    chassis.moveToPoint(-38.084, -46.881, 1000, {.forwards = true});
    chassis.moveToPoint(-30.084, -46.881, 1000, {.forwards = false});
   
}


inline void elim_autonPeriod(IntakeHandler& intake, ScorerHandler& scorer, adi::DigitalOut& loaderPiston,  adi::DigitalOut& scorerPiston, adi::DigitalOut& descorePiston, int goals, bool mirror = false) {
    float m = mirror ? -1.0f : 1.0f;  // Y multiplier
    
    // Starting position 
    chassis.setPose(-54.996, 17.593, mirror ? 180 : 0);

    // Move in front of match loader
    followPath(ELIMPeriodMoveLoadLG1PT1_txt, 12, 3000, true, mirror);

    chassis.turnToHeading(135, 1000);

    // start intake

    intake.setPrimaryOn(true);
    intake.setSecondaryOn(true);
    intake.setSecondaryDirection(true);  // forward
    intake.setPrimaryDirection(true);
    intake.update();

    followPath(ELIMPeriodMoveLoadLG1PT2_txt, 15, 1000, true, mirror);

    // end intake


    followPath(ELIMPeriodMoveLoadLG1PT4_txt, 15, 1000, false, mirror);


    chassis.turnToHeading(mirror ? mirrorHeading(270) : 270, 1000);
    chassis.waitUntilDone();
    // deploy loader
    // retract scorer
    scorerPiston.set_value(false);
    // Go to match loader and load

    intake.setPrimaryOn(true);
    intake.setSecondaryOn(true);
    intake.setSecondaryDirection(true);  // forward
    intake.update();

    followPath(PeriodMoveLoadLG1PT2_txt, 12, 1500, true, mirror);

    chassis.setPose(-56.288, 45.537, 270);
    chassis.moveToPoint(-61.215, 46.209, 800, {.forwards = true});
    chassis.moveToPoint(-60.215, 46.209, 200, {.forwards = false});
    chassis.moveToPoint(-61.215, 46.209, 800, {.forwards = true});

    // chassis.moveToPoint(-59.215, 46.209, 500, {.forwards = false});
    // chassis.moveToPoint(-61.215, 46.209, 500, {.forwards = true});

    chassis.setPose(-61.215, 46.209, 270);

    pros::delay(400);

    intake.setPrimaryOn(false);
    intake.setSecondaryOn(false);
    intake.update();

    // // chassis.setPose(-56.512, 46.657, 270); // RESET POSITION
    // loadBalls(intake, 3000);
    // go TO high goal
    loaderPiston.set_value(false);
    // chassis.setPose(-30.084, 46.881, 270); // RESET POSITION

    // chassis.setPose(-30.66, 47.244, 270);
    scorerPiston.set_value(true);
    followPath(PeriodMoveLoadLG1PT3_txt, 12, 1800, false, mirror);
    intake.setPrimaryOn(true);
    intake.setSecondaryOn(true);
    intake.setSecondaryDirection(true);
    intake.setPrimaryDirection(true);
    scorer.setOn(true);
    scorer.setDirection(true);
    scorer.update();

    intake.update();

    delay(2000);
    // scoreBalls(intake, 3000, false, scorer);
    // Recalibrate position (heading 270, or 90 if mirrored)
    // chassis.setPose(-30.66, 47.244 * (mirror ? -1 : 1), mirror ? mirrorHeading(270) : 270);
    // // // Move forward 17 inches to target
    // chassis.moveToPoint(-47.329, 47.329 * (mirror ? -1 : 1), 2000, {.forwards = true});
    // chassis.waitUntilDone();
    scorerPiston.set_value(false);
    // Move away
    followPath(PeriodMoveLoadLG1PT4_txt, 15, 3000, true, mirror);
    chassis.setPose(-47.932, 47.932, 270);
    chassis.turnToHeading(mirror ? mirrorHeading(135) : 135, 1000);
    chassis.waitUntilDone();

    followPath(ELIMPeriodMoveLoadLG1PT05_txt, 15, 1000, false, mirror);
    chassis.turnToHeading(180, 1000);
    descorePiston.set_value(true);
    followPath(ELIMPeriodMoveLoadLG1PT06_txt, 15, 1000, false, mirror);


}




inline void RELIM_autonPeriod(IntakeHandler& intake, ScorerHandler& scorer, adi::DigitalOut& loaderPiston,  adi::DigitalOut& scorerPiston, adi::DigitalOut& descorePiston, int goals, bool mirror = false) {
    float m = mirror ? -1.0f : 1.0f;  // Y multiplier
    
    // Starting position 
    chassis.setPose(-54.139, -17.246, mirror ? 180 : 180);

    chassis.turnToHeading(135, 1000);

    // Move in front of match loader
    followPath(RELIMPeriodMoveLoadLG1PT1_txt, 12, 3000, true, mirror);

    chassis.turnToHeading(45, 1000);

    // start intake

    intake.setPrimaryOn(true);
    intake.setSecondaryOn(true);
    intake.setSecondaryDirection(false);  // forward
    intake.setPrimaryDirection(false);
    intake.update();

    followPath(RELIMPeriodMoveLoadLG1PT2_txt, 15, 1000, true, mirror);

    // end intake


    followPath(RELIMPeriodMoveLoadLG1PT4_txt, 15, 1000, false, mirror);
    chassis.setPose(-40.881, -46.881, 45);
    loaderPiston.set_value(true);



    chassis.turnToHeading(mirror ? mirrorHeading(270) : 270, 1000);
    chassis.waitUntilDone();

    // deploy loader
    // retract scorer
    scorerPiston.set_value(false);
    // Go to match loader and load

    intake.setPrimaryOn(true);
    intake.setSecondaryOn(true);
    intake.setSecondaryDirection(false);  // forward
    intake.update();

    followPath(RIGHTPeriodMoveLoadLG1PT2_txt, 15, 1500, true, mirror);

    chassis.setPose(-56.288, -45.537, 270);

    // chassis.moveToPoint(-59.215, 46.209, 500, {.forwards = false});
    // chassis.moveToPoint(-61.215, 46.209, 500, {.forwards = true});


    pros::delay(1700);

    intake.setPrimaryOn(false);
    intake.setSecondaryOn(false);
    intake.update();

    // // chassis.setPose(-56.512, 46.657, 270); // RESET POSITION
    // loadBalls(intake, 3000);
    // go TO high goal
    loaderPiston.set_value(false);
    // chassis.setPose(-30.084, 46.881, 270); // RESET POSITION

    // chassis.setPose(-30.66, 47.244, 270);
    scorerPiston.set_value(true);
    followPath(RIGHTPeriodMoveLoadLG1PT3_txt, 15, 1800, false, mirror);
    intake.setPrimaryOn(true);
    intake.setSecondaryOn(true);
    intake.setSecondaryDirection(false);
    intake.setPrimaryDirection(false);
    scorer.setOn(true);
    scorer.setDirection(true);
    scorer.update();

    intake.update();

    delay(2000);
    // scoreBalls(intake, 3000, false, scorer);
    // Recalibrate position (heading 270, or 90 if mirrored)
    // chassis.setPose(-30.66, 47.244 * (mirror ? -1 : 1), mirror ? mirrorHeading(270) : 270);
    // // // Move forward 17 inches to target
    // chassis.moveToPoint(-47.329, 47.329 * (mirror ? -1 : 1), 2000, {.forwards = true});
    // chassis.waitUntilDone();
    descorePiston.set_value(true);
    // Move away
    // chassis.setPose(-47.932, -47.932, 270);
    // chassis.turnToHeading(mirror ? mirrorHeading(135) : 135, 1000);
    // chassis.waitUntilDone();
    followPath(RELIMPeriodMoveLoadLG1PT05_txt, 15, 1000, true, mirror);
    chassis.turnToHeading(0, 1000);
    chassis.moveToPoint(-40.162, -60.647, 1000, {.forwards = false});
    chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(-19.276, -55, 1000, {.forwards = false});
    chassis.turnToHeading(280, 1000);
    chassis.moveToPoint(-20, -55, 1000, {.forwards = false});
    chassis.waitUntilDone();
}
