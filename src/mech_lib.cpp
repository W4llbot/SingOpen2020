#include "main.h"
#include "mech_lib.hpp"

#define LOW_TOWER_RIGHT 270
#define HIGH_TOWER_HEIGHT 400

Motor lRoller(lRollerPort);
Motor rRoller(rRollerPort);
Motor tray(trayPort);
Motor arm(armPort);

int trayTarget = 0;
int armTarget = -30;

void intake(int speed) {
  lRoller.move(speed);
  rRoller.move(speed);
}

void trayControl(void * ignore) {
  double kP = 0.20, rampAmount = 3;
  double integral = 0, speed = 0,  prevError = trayTarget - tray.get_position();

  tray.tare_position();
  trayTarget = 10;

  while(true) {
    // pi control
    double error = trayTarget - tray.get_position();

    double targetSpeed = error*kP;

    // ramping (only one directional)
    if(speed - targetSpeed > rampAmount) speed -= rampAmount;
    else speed = targetSpeed;

    // apply motion
    tray.move(speed);

    // set prev values
    prevError = error;

    // debug
    // printf("%.2f\n", error);

    delay(25);
  }
}

void lowerTray() {
  trayTarget = 0;
}

void raiseTray() {
  trayTarget = 700;
}

void armControl(void * ignore) {
  arm.tare_position();

  while(true) {
    double error = armTarget - arm.get_position();
    arm.move(error);

    delay(25);
  }
}

void lowerCube() {
  intake(-80);
  delay(300);
  intake(0);
}

void lowTower() {
  // lowerCube();
  armTarget = LOW_TOWER_RIGHT;
}

void highTower() {
  // lowerCube();
  armTarget = HIGH_TOWER_HEIGHT;
}

void armDown() {
  armTarget = -30;
}
