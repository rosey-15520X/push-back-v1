#include "main.h"
#include "modules/drive.h"
#include "modules/intake.h"
#include "modules/pneumatics.h"
#include "modules/scorer.h"
#include "modules/state.h"
#include "lemlib/asset.hpp"
#include <string>

// Declare path assets (files must be in static/ folder)
ASSET(GOTOLONGGOAL_txt)
ASSET(GETBALL_txt)

// Motor ports
#define INTAKE_BASE_PRIMARY -7
#define INTAKE_BASE_SECONDARY 18
#define SCORER_LIFT_PORT 1

// Pneumatics ADI ports (3-wire)
#define BLOCK_PISTON_PORT 'G'
#define MIDDLE_GOAL_PISTON_PORT 'H'
#define LOADER_PISTON_PORT 'F'
#define ARM_PISTON_PORT 'E'

using namespace pros;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  lcd::initialize();
  lcd::print(0, "Calibrating IMU...");
  
  // Calibrate chassis IMU - IMPORTANT: do this here so it's ready for auton
  chassis.calibrate();
  
  lcd::print(0, "IMU Calibrated!");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    chassis.turnToHeading(90, 5000);
    chassis.turnToHeading(0, 100000);
}



/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // Initialize state
  State state;

  // Initialize motors
  Motor intakeBasePrimary(INTAKE_BASE_PRIMARY, MOTOR_GEARSET);
  Motor intakeBaseSecondary(INTAKE_BASE_SECONDARY, MOTOR_GEARSET);
  Motor scorerLift(SCORER_LIFT_PORT, MOTOR_GEARSET);

  // Initialize pneumatics (ADI digital outputs)
  adi::DigitalOut blockPiston(BLOCK_PISTON_PORT);
  adi::DigitalOut middleGoalPiston(MIDDLE_GOAL_PISTON_PORT);
  adi::DigitalOut loaderPiston(LOADER_PISTON_PORT);
  adi::DigitalOut armPiston(ARM_PISTON_PORT);

  // Initialize handlers
  ; IntakeHandler intake(intakeBasePrimary, intakeBaseSecondary, state.intake);
  ScorerHandler scorer(scorerLift, state.scorer);
  PneumaticsHandler pneumatics(blockPiston, middleGoalPiston, loaderPiston, armPiston, state.scorer, state.intake, state.loader, state.arm);

  // Initialize pneumatics to match state
  pneumatics.init();

  Controller master(E_CONTROLLER_MASTER);

  master.set_text(0, 0, "INITIALIZED   ");
  master.set_text(1, 0, "READY   ");
  master.set_text(2, 0, "15520X        ");

  int cycle = 0;

  while (true) {
    if (cycle % 25 == 0) {
      master.set_text(0, 0, "INITIALIZED   ");
      master.set_text(2, 0, "15520X        ");
    }

    cycle++;

    // Drive control
    double forward = master.get_analog(ANALOG_LEFT_Y);
    double turn = master.get_analog(ANALOG_RIGHT_X);

    // if (!state.driveDirection) {
    // 	forward = -forward;
    // 	// turn = -turn;

    // 	if (cycle % 25 == 0) {
    // 		// master.set_text(1, 8, "F" + std::to_string(round(forward *
    // 100) / 100) + " T" + std::to_string(round(turn * 100) / 100));
    // 	}
    // }

    // master.set_text(2, 0, "F" + std::to_string(round(forward * 100) / 100) +
    // " T" + std::to_string(round(turn * 100) / 100));

    // Toggle drive direction
    if (master.get_digital_new_press(DIGITAL_Y)) {
      master.clear_line(1);
      master.set_text(1, 0, "SCORER_REVERSE");
      scorer.toggleDirection();
    }

    // Intake controls
    if (master.get_digital_new_press(DIGITAL_R1)) {
      master.clear_line(1);
      master.set_text(1, 0, "INTAKE_TOGGLE");
      intake.toggle();
    }

    if (master.get_digital_new_press(DIGITAL_A)) {
      master.clear_line(1);
      master.set_text(1, 0, "INTAKE_REVERSE");
      intake.toggleDirection();
    }

    // Scorer controls
    // if (master.get_digital_new_press(DIGITAL_L1)) {
    // 	master.set_text(1, 0, "SCR_REV  ");
    // 	scorer.toggleDirection();
    // }

    if (master.get_digital_new_press(DIGITAL_R2)) {
      master.clear_line(1);
      master.set_text(1, 0, "SCORER_TOGGLE");
      scorer.toggle();
    }

    // Pneumatics controls
    if (master.get_digital_new_press(DIGITAL_L2)) {
      master.clear_line(1);
      master.set_text(1, 0, "BLOCK_TOGGLE");
      pneumatics.toggleBlock();
    }

    if (master.get_digital_new_press(DIGITAL_B)) {
      master.clear_line(1);
      master.set_text(1, 0, "MIDDLE_GOAL_TOGGLE");
      pneumatics.toggleMiddleGoal();
    }

    if (master.get_digital_new_press(DIGITAL_L1)) {
      master.clear_line(1);
      master.set_text(1, 0, "LOADER_TOGGLE");
      pneumatics.toggleLoader();
    }

    if (master.get_digital_new_press(DIGITAL_LEFT)) {
      master.clear_line(1);
      master.set_text(1, 0, "ARM TOGGLE");
      pneumatics.toggleArm();
    }

    // Update all handlers
    intake.update();
    scorer.update();

    // Drive
    chassis.arcade(forward, turn);

    // Update LCD with current state
    // updateLCD(state);

    pros::delay(20);
  }
}