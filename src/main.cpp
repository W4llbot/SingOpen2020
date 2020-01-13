#include "main.h"
#include "autons.hpp"
#include "base_lib.hpp"
#include "mech_lib.hpp"
#include "driver_funcs.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
	Motor FL(FLPort, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
	Motor BL(BLPort, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
	Motor FR(FRPort, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
	Motor BR(BRPort, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
	Motor arm(armPort, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_DEGREES);
	Motor lRoller(lRollerPort, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_DEGREES);
	Motor rRoller(rRollerPort, E_MOTOR_GEARSET_36, true, E_MOTOR_ENCODER_DEGREES);
	Motor tray(trayPort, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_DEGREES);
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
	Task basOdom(baseOdometry, NULL, "");
	Task basControl(baseControl, NULL, "");
	Task basMotorControl(baseMotorControl, NULL, "");
	Task trayController(trayControl, NULL, "");
	Task armController(armControl, NULL, "");

	redBack();
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
	Task trayController(trayControl, NULL, "");
	Task armController(armControl, NULL, "");
	Task basOdom(baseOdometry, NULL, "");
	Controller master(E_CONTROLLER_MASTER);

	double left;
	double right;
	bool armUp = false;
	bool isTank = true;
	while(true) {
		if(master.get_digital_new_press(DIGITAL_Y)) {
			isTank = !isTank;
		}

		if(isTank) {
			left = master.get_analog(ANALOG_LEFT_Y);
			right = master.get_analog(ANALOG_RIGHT_Y);
		}else {
			double power = master.get_analog(ANALOG_LEFT_Y);
			double turn = master.get_analog(ANALOG_RIGHT_X);

			left = power + turn;
			right = power - turn;
		}

		base(left, right);

		if(master.get_digital(DIGITAL_R1)) {
			intake(127);
		}else if(master.get_digital(DIGITAL_R2)) {
			intake(-127);
		}else {
			intake(0);
		}

		if(master.get_digital(DIGITAL_X)) {
			raiseTray();
		}else if(master.get_digital(DIGITAL_A)) {
			releaseCubes();
		}

		if(master.get_digital_new_press(DIGITAL_L1)) {
			if(armUp) {
				armDown();
				armUp = false;
			}else {
				lowTower();
				armUp = true;
			}
		}else if(master.get_digital_new_press(DIGITAL_L2)) {
			if(armUp) {
				armDown();
				armUp = false;
			}else {
				highTower();
				armUp = true;
			}
		}

		delay(25);
	}
}
