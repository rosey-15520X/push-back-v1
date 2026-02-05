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

/**
 * Autonomous routine for Skills
 * Full 1-minute skills run
 * @param intake - IntakeHandler reference
 * @param loaderPiston - Loader piston reference
 */
inline void autonSkills(IntakeHandler& intake, pros::adi::DigitalOut& loaderPiston) {
    // Starting position (top left corner)
    chassis.setPose(-62.097, 17.466, 0);

    // ========== PHASE 1: First match loader (top left) ==========
    // Move to first match loader
    chassis.follow(SkillsMoveLoadLG1_txt, 15, 3000, true);
    chassis.waitUntilDone();
    chassis.turnToHeading(270, 1000);
    chassis.waitUntilDone();
    
    // Align with loader
    chassis.follow(SkillsMoveLoadLG1PT2_txt, 15, 2000, true);
    chassis.waitUntilDone();
    
    // Load balls from match loader
    startMatchLoader(loaderPiston, intake);
    pros::delay(500);
    pushMatchLoader(loaderPiston);
    pros::delay(1000);
    resetMatchLoader(loaderPiston);
    pros::delay(500);
    pushMatchLoader(loaderPiston);
    pros::delay(1000);
    resetMatchLoader(loaderPiston);
    pros::delay(500);
    pushMatchLoader(loaderPiston);
    pros::delay(1000);
    stopMatchLoader(loaderPiston, intake);
    
    // ========== PHASE 2: Score at top right goal ==========
    // Arc across top to right side
    chassis.follow(SkillsMoveLoadLG1PT3_txt, 15, 5000, true);
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 1000);
    chassis.waitUntilDone();
    
    // Score balls into goal
    scoreBalls(intake, 2000, false);  // High goal
    
    // Move to right side loader
    chassis.follow(SkillsMoveLoadLG1PT4_txt, 15, 2000, true);
    chassis.waitUntilDone();
    
    // Load more balls from right match loader
    startMatchLoader(loaderPiston, intake);
    pros::delay(500);
    pushMatchLoader(loaderPiston);
    pros::delay(1000);
    resetMatchLoader(loaderPiston);
    pros::delay(500);
    pushMatchLoader(loaderPiston);
    pros::delay(1000);
    resetMatchLoader(loaderPiston);
    pros::delay(500);
    pushMatchLoader(loaderPiston);
    pros::delay(1000);
    stopMatchLoader(loaderPiston, intake);
    
    // ========== PHASE 3: Return and score middle ==========
    chassis.follow(SkillsMoveLoadLG1PT5_txt, 15, 3000, false);
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 1000);
    chassis.waitUntilDone();
    
    // Score at middle goal
    scoreBalls(intake, 1500, true);  // Middle goal
    
    // ========== PHASE 4: Complex path to bottom area ==========
    chassis.follow(SkillsMoveLoadLG1PT6_txt, 15, 6000, true);
    chassis.waitUntilDone();
    chassis.turnToHeading(135, 1000);
    chassis.waitUntilDone();
    
    // Score at bottom middle
    scoreBalls(intake, 1500, false);
    
    // ========== PHASE 5: Move to bottom right loader ==========
    chassis.follow(SkillsMoveLoadLG1PT8_txt, 15, 4000, true);
    chassis.waitUntilDone();
    chassis.turnToHeading(270, 1000);
    chassis.waitUntilDone();
    
    // Load from bottom right loader
    startMatchLoader(loaderPiston, intake);
    pros::delay(500);
    pushMatchLoader(loaderPiston);
    pros::delay(1000);
    resetMatchLoader(loaderPiston);
    pros::delay(500);
    pushMatchLoader(loaderPiston);
    pros::delay(1000);
    resetMatchLoader(loaderPiston);
    pros::delay(500);
    pushMatchLoader(loaderPiston);
    pros::delay(1000);
    stopMatchLoader(loaderPiston, intake);
    
    // ========== PHASE 6: Arc to bottom left ==========
    chassis.follow(SkillsMoveLoadLG1PT9_txt, 15, 5000, true);
    chassis.turnToHeading(270, 1000);
    
    // Score at bottom left
    scoreBalls(intake, 2000, false);
    
    // ========== PHASE 7: Final position ==========
    chassis.follow(SkillsMoveLoadLG1PT10_txt, 15, 2000, false);
    
    // Load from bottom left loader
    startMatchLoader(loaderPiston, intake);
    pros::delay(500);
    pushMatchLoader(loaderPiston);
    pros::delay(1000);
    resetMatchLoader(loaderPiston);
    pros::delay(500);
    pushMatchLoader(loaderPiston);
    pros::delay(1000);
    stopMatchLoader(loaderPiston, intake);
    
    // Final score
    scoreBalls(intake, 2000, false);
}
