#pragma once

#include "main.h"
#include "../modules/intake.h"
#include "../modules/scorer.h"
#include <cstring>
#include <cstdlib>
#include <cstdio>

/**
 * Autonomous helper functions for ball handling
 * These require an IntakeHandler reference to be passed in
 */

// ============================================================================
// PATH MIRRORING - Mirror paths across X-axis (negate Y coordinates)
// ============================================================================

// Static buffer for mirrored path data (max 8KB should be plenty)
static char mirroredPathBuffer[8192];

/**
 * Mirror a LemLib path by negating Y coordinates
 * Works with Path.jerryio format: "x, y, speed" per line
 * Returns pointer to static buffer - use immediately or copy!
 */
inline const char* mirrorPath(const char* pathData) {
    char* out = mirroredPathBuffer;
    const char* in = pathData;
    
    while (*in) {
        // Check if this line starts with a number (path point) or minus sign
        if ((*in >= '0' && *in <= '9') || *in == '-') {
            // Parse x value
            char* endPtr;
            float x = strtof(in, &endPtr);
            in = endPtr;
            
            // Skip comma and whitespace
            while (*in == ',' || *in == ' ') in++;
            
            // Parse y value and NEGATE it
            float y = -strtof(in, &endPtr);
            in = endPtr;
            
            // Skip comma and whitespace
            while (*in == ',' || *in == ' ') in++;
            
            // Parse speed value (keep as-is)
            float speed = strtof(in, &endPtr);
            in = endPtr;
            
            // Write mirrored line
            out += sprintf(out, "%.3f, %.3f, %.3f", x, y, speed);
        }
        
        // Copy rest of line (including newline)
        while (*in && *in != '\n') {
            *out++ = *in++;
        }
        if (*in == '\n') {
            *out++ = *in++;
        }
    }
    *out = '\0';
    
    return mirroredPathBuffer;
}

/**
 * Mirror a heading (for turnToHeading calls)
 * Flips across X-axis: 90 -> 270, 45 -> 315, etc.
 */
inline float mirrorHeading(float heading) {
    float mirrored = 360.0f - heading;
    if (mirrored >= 360.0f) mirrored -= 360.0f;
    if (mirrored < 0.0f) mirrored += 360.0f;
    return mirrored;
}

inline void loadBalls(IntakeHandler& intake, int time) {
    intake.setPrimaryOn(true);
    intake.setSecondaryOn(true);
    intake.setSecondaryDirection(true);  // forward
    intake.update();
    pros::delay(time);
    intake.setPrimaryOn(false);
    intake.setSecondaryOn(false);
    intake.update();
}

// Start intake in background - keeps running during path following
inline pros::Task* startIntakeTask(IntakeHandler& intake, bool forward = true) {
    intake.setPrimaryOn(true);
    intake.setSecondaryOn(true);
    intake.setSecondaryDirection(forward);
    intake.setPrimaryDirection(forward);
    
    return new pros::Task([](void* param) {
        IntakeHandler* i = static_cast<IntakeHandler*>(param);
        while (true) {
            i->update();
            pros::delay(20);
        }
    }, &intake, "intakeTask");
}

inline void stopIntakeTask(pros::Task* task, IntakeHandler& intake) {
    if (task) {
        task->remove();
        delete task;
    }
    intake.setPrimaryOn(false);
    intake.setSecondaryOn(false);
    intake.update();
}

inline void shootBalls(IntakeHandler& intake, int time) {
    intake.setPrimaryOn(true);
    intake.setSecondaryOn(true);
    intake.setSecondaryDirection(true);  // forward
    intake.update();
    pros::delay(time);
    intake.setPrimaryOn(false);
    intake.setSecondaryOn(false);
    intake.update();
}

inline void shootMiddleBalls(IntakeHandler& intake, int time) {
    intake.setPrimaryOn(true);
    intake.setSecondaryOn(true);
    intake.setSecondaryDirection(false);  // inverse/backward
    intake.update();
    pros::delay(time);
    intake.setPrimaryOn(false);
    intake.setSecondaryOn(false);
    intake.update();
}

/**
 * Start receiving from match loader
 * Turns on intake and sets loader to receive position
 */
inline void startMatchLoader(pros::adi::DigitalOut& loaderPiston, IntakeHandler& intake) {
    // Start intake to receive balls
    intake.setPrimaryOn(true);
    intake.setSecondaryOn(true);
    intake.setSecondaryDirection(true);  // forward to intake
    intake.update();
    
    // Contract loader to receive position
    loaderPiston.set_value(true);
}

/**
 * Push balls from match loader
 * Extends the loader piston to push balls into robot
 */
inline void pushMatchLoader(pros::adi::DigitalOut& loaderPiston) {
    loaderPiston.set_value(false);  // Extend to push
}

/**
 * Reset match loader to receive position
 */
inline void resetMatchLoader(pros::adi::DigitalOut& loaderPiston) {
    loaderPiston.set_value(true);  // Contract to receive
}

/**
 * Stop match loader
 * Turns off intake and leaves loader contracted
 */
inline void stopMatchLoader(pros::adi::DigitalOut& loaderPiston, IntakeHandler& intake) {
    // Stop intake
    intake.setPrimaryOn(false);
    intake.setSecondaryOn(false);
    intake.update();
    
    // Leave loader contracted (ready position)
    loaderPiston.set_value(true);
}

/**
 * Score balls into a goal
 * @param intake - Reference to the intake handler
 * @param time - Time to run scorer in ms
 * @param middleGoal - If true, shoot backward for middle goal
 */
inline void scoreBalls(IntakeHandler& intake, int time, bool middleGoal, ScorerHandler& scorer, pros::adi::DigitalOut& scorerPiston) {
    intake.setPrimaryOn(true);
    intake.setSecondaryOn(true);
    intake.setSecondaryDirection(true);
    intake.setPrimaryDirection(true);
    scorerPiston.set_value(true);
    scorer.setOn(true);
    scorer.setDirection(!middleGoal);
    scorer.update();
    intake.update();
    pros::delay(time);
    intake.setPrimaryOn(false);
    intake.setSecondaryOn(false);
    scorer.setOn(false);
    intake.update();
    scorer.update();
    scorerPiston.set_value(false);
}
