#pragma once

#include "main.h"
#include "../modules/intake.h"
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
