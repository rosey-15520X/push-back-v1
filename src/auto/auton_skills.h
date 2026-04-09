#pragma once

#include "main.h"
#include "lemlib/asset.hpp"
#include "auton_helpers.h"
#include "../modules/drive.h"

// Path assets for Skills auton
ASSET(SkillsMoveLoadLG1_txt)
ASSET(SkillsMoveLoadLG1PT2_txt)
ASSET(SkillsMoveLoadLG1PT3_txt)
ASSET(SkillsMoveLoadLG1PT4_txt)
ASSET(SkillsMoveLoadLG1PT5_txt)
ASSET(SkillsMoveLoadLG1PT6_txt)
ASSET(SkillsMoveLoadLG1PT7_txt)
ASSET(SkillsMoveLoadLG1PT8_txt)
ASSET(SkillsMoveLoadLG1PT9_txt)
ASSET(SkillsMoveLoadLG1PT10_txt)
ASSET(SkillsMoveLoadLG1PT11_txt)
ASSET(SkillsMoveLoadLG1PT12_txt)

/**
 * Autonomous routine for Skills
 * Full 1-minute skills run
 * @param intake - IntakeHandler reference
 * @param scorer - ScorerHandler reference
 * @param loaderPiston - Loader piston reference
 */
inline void autonSkills(IntakeHandler& intake, ScorerHandler& scorer, pros::adi::DigitalOut& loaderPiston, pros::adi::DigitalOut& scorerPiston, pros::Distance& distanceSensor) {
    // Starting position (top left corner)
    chassis.setPose(-54.996, 17.593, 0);

    // PHASE 1.0 START
    chassis.follow(SkillsMoveLoadLG1_txt, 15, 3000, true);
    chassis.waitUntilDone();
    loaderPiston.set_value(true);
    delay(500);

    chassis.turnToHeading(270, 1000);
    chassis.waitUntilDone();
    
    // MARK LOADER
    intake.setPrimaryOn(true);
    intake.setSecondaryOn(true);
    intake.setSecondaryDirection(true);  // forward
    intake.setPrimaryDirection(true);  // forward
    intake.update();
    chassis.follow(SkillsMoveLoadLG1PT2_txt, 15, 2000, true);
    chassis.waitUntilDone();
    // // PHASE 1.1 LOAD
    chassis.setPose(-56.288, 47.329, 270);
    chassis.moveToPoint(-69.215, 47.329, 800, {.forwards = true}); // OG PT
    chassis.setPose(-56.581, 46.947, 270);

    chassis.waitUntilDone();

    delay(1000);

    intake.setPrimaryOn(false);
    intake.setSecondaryOn(false);
    intake.update();

    loaderPiston.set_value(false);


    // PHASE 1.2 TRV LOADER
    chassis.follow(SkillsMoveLoadLG1PT3_txt, 15, 10000, false);
    chassis.waitUntilDone();

    // RESET POSITION
    // distance snesor port 10
    // // get distance from distance sensor
    // int distance = distanceSensor.get_distance() / 25.4;
    
    // chassis.setPose(47.41, 140 - distance - 1, 180);
    // chassis.moveToPoint(47.41, 46.433, 1000, {.forwards = true});
    

    chassis.turnToHeading(90, 1000);
    chassis.waitUntilDone();
    chassis.setPose(47.41, 46.433, 90);

    chassis.waitUntilDone();

    chassis.moveToPoint(22.164, 46.433, 1000, {.forwards = false});
    chassis.waitUntilDone();
    scoreBalls(intake, 2000, false, scorer, scorerPiston);

    loaderPiston.set_value(true);


    chassis.moveToPoint(47.41, 46.433, 1000, {.forwards = true});
    // chassis.setPose(47.41, 46.433, 90);
    chassis.waitUntilDone();


    // PHASE 1.3 LOAD
    intake.setPrimaryOn(true);
    intake.setSecondaryOn(true);
    intake.setSecondaryDirection(true);  // forward
    intake.setPrimaryDirection(true);  // forward
    intake.update();
    // Recalibrate odometry + slow approach to loader wall
    // chassis.setPose(55.025, 46.209, 90);
    chassis.moveToPoint(69.296, 46.209, 1000, {.forwards=true});
    chassis.setPose(61.296, 46.209, 90);
    chassis.waitUntilDone();
    delay(1000);
    loaderPiston.set_value(false);
    intake.setPrimaryOn(false);
    intake.setSecondaryOn(false);
    intake.update();
    
    // PHASE 1.4 SCORE
    chassis.follow(SkillsMoveLoadLG1PT5_txt, 15, 3000, false);
    chassis.waitUntilDone();
    chassis.setPose(30.304, 47.071, 90);
    scoreBalls(intake, 2000, false, scorer, scorerPiston);
    
    // PHASE 1~2.0 MOVE AWAY FROM LDL
    chassis.follow(SkillsMoveLoadLG1PT6_txt, 15, 1000, true);
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 1000);
    chassis.waitUntilDone();

    loaderPiston.set_value(true);

    // PHASE 1~2.1 MOVE TO PNT
    chassis.follow(SkillsMoveLoadLG1PT7_txt, 15, 6000, true);
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 1000);
    chassis.waitUntilDone();
    
    // PHASE 2.2 LOAD
    intake.setPrimaryOn(true);
    intake.setSecondaryOn(true);
    intake.setSecondaryDirection(true);  // forward
    intake.setPrimaryDirection(true);  // forward
    intake.update();
    // MARK LOADER
    // Recalibrate odometry + slow approach to loader wall
    // chassis.setPose(55.249, -48.082, 90);
    chassis.moveToPoint(69, -47.41, 1500, {.forwards=true});
    chassis.waitUntilDone();
    chassis.setPose(56.628, -47.41, 90);
    chassis.waitUntilDone();
    delay(1000);
    intake.setPrimaryOn(false);
    intake.setSecondaryOn(false);
    intake.update();

    // PHASE 2.3 MOVE FROM LDL
    chassis.moveToPoint(47.858, -47.41, 3000, {.forwards=false});
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 1000);
    chassis.waitUntilDone();
    loaderPiston.set_value(false);


    chassis.follow(SkillsMoveLoadLG1PT9_txt, 15, 7000, true);
    chassis.waitUntilDone();
    chassis.turnToHeading(270, 1000);
    chassis.waitUntilDone();

    chassis.moveToPoint(29.94, -47.41, 3000, {.forwards=false});
    chassis.waitUntilDone();
    loaderPiston.set_value(true);
    scoreBalls(intake, 2000, false, scorer, scorerPiston);
    intake.setPrimaryOn(true);
    intake.setSecondaryOn(true);
    intake.setSecondaryDirection(true);  // forward
    intake.setPrimaryDirection(true);  // forward
    intake.update();
    chassis.moveToPoint(-46.657, -47.41, 3000, {.forwards=true});
    chassis.waitUntilDone();
    loaderPiston.set_value(true);
    chassis.moveToPoint(-63.455, -47.41, 1000, {.forwards=true});
    chassis.waitUntilDone();
    delay(1000);
    loaderPiston.set_value(false);
    intake.setPrimaryOn(false);
    intake.setSecondaryOn(false);
    intake.update();

    // SCORE
    chassis.follow(SkillsMoveLoadLG1PT11_txt, 15, 3000, false);
    chassis.waitUntilDone();
    scoreBalls(intake, 2000, false, scorer, scorerPiston);

    // PARK
    chassis.follow(SkillsMoveLoadLG1PT12_txt, 15, 3000, true);
    chassis.waitUntilDone();

    

}
