#include "main.h"
#include "modules/drive.h"
#include "modules/intake.h"
#include "modules/pneumatics.h"
#include "modules/scorer.h"
#include "modules/state.h"
#include "auto/auton_period.h"
#include "pros/misc.h"

// ============================================================================
// AUTON SELECTOR
// FLAG: Selector is shown in initialize() so it's always available, even 
//       without a competition switch. This allows testing autons anytime.
//       Move to competition_initialize() if you only want it during matches.
// ============================================================================
int autonSelector = 0;
bool isRedAlliance = true;
bool autonConfirmed = false;  // Must confirm before robot runs (without comp switch)

const char* autonNames[] = {"Left 1 Goal", "Left 2 Goals", "Left 3 Goals", "Right 1 Goal", "Right 2 Goals", "Right 3 Goals", "Skills", "None"};
const int AUTON_COUNT = 8;

// Controller for auton selection
pros::Controller selectorController(pros::E_CONTROLLER_MASTER);

void updateAutonDisplay() {
    pros::lcd::print(1, "< %s >", autonNames[autonSelector]);
    pros::lcd::print(2, "Alliance: %s", isRedAlliance ? "RED" : "BLUE");
    if (!autonConfirmed) {
        pros::lcd::print(3, "L1/R1=sel A=color B=OK");
        selectorController.print(0, 0, "<%s>", autonNames[autonSelector]);
        selectorController.print(1, 0, "%s  B=Confirm", isRedAlliance ? "RED" : "BLU");
    } else {
        pros::lcd::print(3, ">> CONFIRMED <<");
        selectorController.print(0, 0, "CONFIRMED: %s", autonNames[autonSelector]);
        selectorController.print(1, 0, "%s", isRedAlliance ? "RED ALLIANCE" : "BLUE ALLIANCE");
    }
}

// Motor ports
#define INTAKE_BASE_PRIMARY -7
#define INTAKE_BASE_SECONDARY -4
#define SCORER_LIFT_PORT 4

// Pneumatics ADI ports (3-wire)
#define BLOCK_PISTON_PORT 'C'
#define LOADER_PISTON_PORT 'B'
#define ARM_PISTON_PORT 'A'

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
void competition_initialize() {
  lcd::print(0, "[PROMPT] SELECT AUTON:");
  updateAutonDisplay();

  // Controller-based auton selector (runs in background)
  pros::Task selectorTask([](void*) {
    while (!autonConfirmed) {
      // L1 = previous auton
      if (selectorController.get_digital_new_press(DIGITAL_L1)) {
        autonSelector = (autonSelector - 1 + AUTON_COUNT) % AUTON_COUNT;
        updateAutonDisplay();
      }
      // R1 = next auton
      if (selectorController.get_digital_new_press(DIGITAL_R1)) {
        autonSelector = (autonSelector + 1) % AUTON_COUNT;
        updateAutonDisplay();
      }
      // A = toggle alliance color
      if (selectorController.get_digital_new_press(DIGITAL_A)) {
        isRedAlliance = !isRedAlliance;
        updateAutonDisplay();
      }
      // B = confirm selection
      if (selectorController.get_digital_new_press(DIGITAL_B)) {
        autonConfirmed = true;
        updateAutonDisplay();
        selectorController.rumble("-");  // Short rumble to confirm
      }
      pros::delay(20);
    }
  }, nullptr, "selectorTask");
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
void initialize() {
  lcd::initialize();
  // Print before calibration so screen isn't blank
  lcd::print(0, "[INFO] IMU calibrating...");

  // Calibrate WITHOUT risking a permanent block.
  // NOTE: In some LemLib/PROS setups, chassis.calibrate() can block forever if the IMU
  // isn't present/wired correctly. Run it in a task so we can still time out.
  pros::Task imuCalTask([](void*) { chassis.calibrate(); }, nullptr, "imuCal");

  int timeoutMs = 6000;
  // Give the calibrate task a moment to kick off
  pros::delay(50);
  while (imu.is_calibrating() && timeoutMs > 0) {
    pros::delay(20);
    timeoutMs -= 20;
  }

  if (imu.is_calibrating()) {
    lcd::print(0, "[WARN] IMU timeout");
  } else {
    lcd::print(0, "[INFO] Ready");
  }
  
  // // Show auton selector after calibration
  // lcd::print(0, "[PROMPT] SELECT AUTON:");
  // updateAutonDisplay();

  // // Controller-based auton selector (runs in background)
  // pros::Task selectorTask([](void*) {
  //   while (!autonConfirmed) {
  //     // L1 = previous auton
  //     if (selectorController.get_digital_new_press(DIGITAL_L1)) {
  //       autonSelector = (autonSelector - 1 + AUTON_COUNT) % AUTON_COUNT;
  //       updateAutonDisplay();
  //     }
  //     // R1 = next auton
  //     if (selectorController.get_digital_new_press(DIGITAL_R1)) {
  //       autonSelector = (autonSelector + 1) % AUTON_COUNT;
  //       updateAutonDisplay();
  //     }
  //     // A = toggle alliance color
  //     if (selectorController.get_digital_new_press(DIGITAL_A)) {
  //       isRedAlliance = !isRedAlliance;
  //       updateAutonDisplay();
  //     }
  //     // B = confirm selection
  //     if (selectorController.get_digital_new_press(DIGITAL_B)) {
  //       autonConfirmed = true;
  //       updateAutonDisplay();
  //       selectorController.rumble("-");  // Short rumble to confirm
  //     }
  //     pros::delay(20);
  //   }
  // }, nullptr, "selectorTask");
}

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
  // Auto-confirm if running in competition mode
  autonConfirmed = true;
  
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

  // Wait for auton selection confirmation if no competition switch
  // if (!pros::competition::is_connected()) {
  //   lcd::print(0, "[PROMPT] SELECT AUTON:");
  //   while (!autonConfirmed) {
  //     pros::delay(50);  // Wait for user to confirm selection
  //   }
    
  //   // Ask: run auton or driver?
  //   lcd::print(0, "[PROMPT] RUN AUTON?");
  //   lcd::print(3, "L1=Auton  R1=Driver");
    
  //   bool waitingForChoice = true;
  //   bool runAuton = false;
    
  //   while (waitingForChoice) {
  //     if (robot.master.get_digital_new_press(DIGITAL_L1)) {
  //       runAuton = true;
  //       waitingForChoice = false;
  //     }
  //     if (robot.master.get_digital_new_press(DIGITAL_R1)) {
  //       runAuton = false;
  //       waitingForChoice = false;
  //     }
  //     pros::delay(50);
  //   }
    
  //   if (runAuton) {
  //     lcd::print(0, "[INFO] Running Auton...");
  //     lcd::print(3, "                        ");
  //     autonomous();  // Run the selected auton
  //     lcd::print(0, "[INFO] Auton Complete!");
  //     pros::delay(1000);
  //   }
    
  //   lcd::print(0, "[INFO] Driver Control");
  //   lcd::print(3, "                        ");
  // }
  lcd::print(0, "[INFO] Driver Control");
  // chassis.setPose(0, 0, 0);
  // chassis.moveToPoint(0, 24, 3000);

  int cycle = 0;

  while (true) {
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

    // // HANDLE TOP OPTICAL SENSOR
    // int hue = robot.opticalSensorTop.get_hue();
    // bool isRed = hue < 30 || hue > 330;
    // bool isBlue = hue < 240;

    // // IMPORTANT: Don't `continue;` here — that skips driver control + drivetrain update.
    // // Only run the reject/eject logic when we see an opponent ball.
    // bool opponentBall = (isRed && !isRedAlliance) || (isBlue && isRedAlliance);
    // if (opponentBall) {
    //   if (robot.state.scorer.liftOn) {
    //     robot.scorer.toggleDirection();
    //     delay(800);
    //     robot.scorer.toggleDirection();
    //   } else {
    //     robot.scorer.setOn(true);
    //     delay(800);
    //     robot.scorer.setOn(false);
    //   }
    // }

    // HANDLE BOTTOM OPTICAL SENSOR
    // int hueBottom = robot.opticalSensorBottom.get_hue();
    // bool isRedBottom = hueBottom < 30 || hueBottom > 330;
    // bool isBlueBottom = hueBottom < 240;

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

    if (robot.master.get_digital_new_press(DIGITAL_R1)) {
      robot.master.clear_line(1);
      robot.master.set_text(1, 0, "LOADER_TOGGLE");
      robot.pneumatics.toggleLoader();
    }

    if (robot.master.get_digital_new_press(DIGITAL_L1)) {
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