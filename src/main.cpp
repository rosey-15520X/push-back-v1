#include "main.h"
#include "modules/drive.h"
#include "modules/intake.h"
#include "modules/pneumatics.h"
#include "modules/scorer.h"
#include "modules/state.h"
#include "auto/auton_period.h"

// ============================================================================
// AUTON SELECTOR
// FLAG: Selector is shown in initialize() so it's always available, even 
//       without a competition switch. This allows testing autons anytime.
//       Move to competition_initialize() if you only want it during matches.
// ============================================================================
int autonSelector = 0;
bool isRedAlliance = true;  // Toggle with middle button

const char* autonNames[] = {"Left 1 Goal", "Left 2 Goals", "Left 3 Goals", "Right 1 Goal", "Right 2 Goals", "Right 3 Goals", "Skills", "None"};
const int AUTON_COUNT = 4;

void updateAutonDisplay() {
    pros::lcd::print(1, "< %s >", autonNames[autonSelector]);
    pros::lcd::print(2, "Alliance: %s", isRedAlliance ? "RED" : "BLUE");
}

void on_left_button() {
    autonSelector = (autonSelector - 1 + AUTON_COUNT) % AUTON_COUNT;
    updateAutonDisplay();
}

void on_center_button() {
    isRedAlliance = !isRedAlliance;
    updateAutonDisplay();
}

void on_right_button() {
    autonSelector = (autonSelector + 1) % AUTON_COUNT;
    updateAutonDisplay();
}

// Motor ports
#define INTAKE_BASE_PRIMARY -7
#define INTAKE_BASE_SECONDARY -4
#define SCORER_LIFT_PORT 9

// Pneumatics ADI ports (3-wire)
#define BLOCK_PISTON_PORT 'G'
#define LOADER_PISTON_PORT 'F'
#define ARM_PISTON_PORT 'E'

#define OPTICAL_PORT_TOP 1
#define OPTICAL_PORT_BOTTOM 2

using namespace pros;

// ============================================================================
// ROBOT STRUCT - Holds all robot components
// ============================================================================
struct Robot {
    State state;
    
    // Motors
    Motor intakeBasePrimary{INTAKE_BASE_PRIMARY, MOTOR_GEARSET};
    Motor intakeBaseSecondary{INTAKE_BASE_SECONDARY, MOTOR_GEARSET};
    Motor scorerLift{SCORER_LIFT_PORT, MOTOR_GEARSET};
    
    // Pneumatics
    adi::DigitalOut blockPiston{BLOCK_PISTON_PORT};
    adi::DigitalOut loaderPiston{LOADER_PISTON_PORT};
    adi::DigitalOut armPiston{ARM_PISTON_PORT};
    
    // Sensors
    Optical opticalSensorTop{OPTICAL_PORT_TOP};
    Optical opticalSensorBottom{OPTICAL_PORT_BOTTOM};
    
    // Handlers (initialized in constructor)
    PneumaticsHandler pneumatics;
    IntakeHandler intake;
    ScorerHandler scorer;
    
    // Controller
    Controller master{E_CONTROLLER_MASTER};
    
    Robot() : 
        pneumatics(blockPiston, loaderPiston, armPiston, state.scorer, state.intake, state.loader, state.arm),
        intake(intakeBasePrimary, intakeBaseSecondary, state.intake),
        scorer(scorerLift, state.scorer) 
    {
        pneumatics.init();
    }
};

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  lcd::initialize();
  lcd::print(0, "[INFO] Initializing......");
  
  // Calibrate chassis IMU - IMPORTANT: do this here so it's ready for auton
  chassis.calibrate();
  
  // Show auton selector after calibration
  lcd::print(0, "[PROMPT] SELECT AUTON:");
  updateAutonDisplay();
  lcd::register_btn0_cb(on_left_button);   // Left button: prev auton
  lcd::register_btn1_cb(on_center_button); // Center button: toggle alliance
  lcd::register_btn2_cb(on_right_button);  // Right button: next auton
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
  Robot robot;
  
  lcd::print(2, "[INFO] AUTON RUNNING: %s", autonNames[autonSelector]);
  chassis.setPose(0, 0, 0);
  
  switch (autonSelector) {
    case 0:  // Left
      autonPeriodLeft(robot.intake, robot.loaderPiston, 1);
      break;
    case 1:  // Left 2 Goals
      autonPeriodLeft(robot.intake, robot.loaderPiston, 2);
      break;
    case 2:  // Left 3 Goals
      autonPeriodLeft(robot.intake, robot.loaderPiston, 3);
      break;
    case 3:  // Right 1 Goal
      autonPeriodRight(robot.intake, robot.loaderPiston, 1);
      break;
    case 4:  // Right 2 Goals
      autonPeriodRight(robot.intake, robot.loaderPiston, 2);
      break;
    case 5:  // Right 3 Goals
      autonPeriodRight(robot.intake, robot.loaderPiston, 3);
      break;
    default:  // None
      break;
  }
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
  Robot robot;
  
  autonPeriodLeft(robot.intake, robot.loaderPiston, 3);
}

void opcontrol1() {
  Robot robot;

  int cycle = 0;
  
  // set position to x:0, y:0, heading:0
  chassis.setPose(0, 0, 0);
  // turn to face heading 90 with a very long timeout
  chassis.turnToHeading(90, 100000);

  while (false == true) {
    cycle++;

    // Drive control
    double forward = robot.master.get_analog(ANALOG_LEFT_Y);
    double turn = robot.master.get_analog(ANALOG_RIGHT_X);

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

    // // Toggle drive direction
    // if (master.get_digital_new_press(DIGITAL_Y)) {
    //   master.clear_line(1);
    //   master.set_text(1, 0, "SCORER_REVERSE");
    //   scorer.toggleDirection();
    // }

    // Intake controls
    if (robot.master.get_digital_new_press(DIGITAL_R1)) {
      robot.master.clear_line(1);
      robot.master.set_text(1, 0, "INTAKE_TOGGLE");
      robot.intake.toggle();
    }

    if (robot.master.get_digital_new_press(DIGITAL_A)) {
      robot.master.clear_line(1);
      robot.master.set_text(1, 0, "SYSTEM_REVERSE");
      robot.intake.toggleDirection();
    }

    int hue = robot.opticalSensorTop.get_hue();
    bool isRed = hue < 30 || hue > 330;
    bool isBlue = hue < 240;
    
    if (isRed) {
      // Detected red
      if (isRedAlliance) continue;
    } else if (isBlue) {
      // Detected blue
      if (!isRedAlliance) continue;

      if (robot.state.scorer.liftOn) { // If currently scoring, shoot the ball away from the goal
        // Lift is facing forward
        robot.scorer.toggleDirection();
        delay(800);
        robot.scorer.toggleDirection();
        // This effectly does the following:
        // 1. Middle goal scoring is in effect: it would inverse the scorer direction super quickly
        //    allowing for the opposition ball to be shot towards the high goal direction, and in turn
        //    prevent the ball from actually scoring. This is done super quickly (~0.8 second)
        // 2. High goal scoring is in effect: it would inverse the scorer direction towards the middle goal
        //    ball would fly downwards, preventing it from scoring
      } else { // If not currently scoring, simply launch the ball away
        // Lift is facing backward
        robot.scorer.setOn(true); // Turn on the scorer to eject the ball
        delay(800);
        robot.scorer.setOn(false); // Turn it off quickly to prevent ejecting alliance balls behind
      }
    }

    // Scorer controls
    // if (master.get_digital_new_press(DIGITAL_L1)) {
    // 	master.set_text(1, 0, "SCR_REV  ");
    // 	scorer.toggleDirection();
    // }

    if (robot.master.get_digital_new_press(DIGITAL_R2)) {
      robot.master.clear_line(1);
      robot.master.set_text(1, 0, "SCORER_TOGGLE");
      robot.scorer.toggle();
    }

    // Pneumatics controls
    if (robot.master.get_digital_new_press(DIGITAL_L2)) {
      robot.master.clear_line(1);
      robot.master.set_text(1, 0, "BLOCK_TOGGLE");
      robot.pneumatics.toggleBlock();
    }

    if (robot.master.get_digital_new_press(DIGITAL_B)) {
      robot.master.clear_line(1);
      robot.master.set_text(1, 0, "MIDDLE_GOAL_TOGGLE");
      robot.scorer.toggleDirection();
    }

    if (robot.master.get_digital_new_press(DIGITAL_L1)) {
      robot.master.clear_line(1);
      robot.master.set_text(1, 0, "LOADER_TOGGLE");
      robot.pneumatics.toggleLoader();
    }

    if (robot.master.get_digital_new_press(DIGITAL_LEFT)) {
      robot.master.clear_line(1);
      robot.master.set_text(1, 0, "ARM TOGGLE");
      robot.pneumatics.toggleArm();
    }

    // Update all handlers
    robot.intake.update();
    robot.scorer.update();

    // Drive
    chassis.arcade(forward, turn);


    pros::delay(20);
  }
}