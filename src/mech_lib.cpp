#include "main.h"
#include "mech_lib.hpp"

#define LOW_TOWER_RIGHT 300
#define HIGH_TOWER_HEIGHT 400

Motor lRoller(lRollerPort);
Motor rRoller(rRollerPort);
Motor tray(trayPort);
Motor arm(armPort);

int trayTarget = 0;
int armTarget = 0;

void intake(int speed) {
  lRoller.move(speed);
  rRoller.move(speed);
}

void trayControl(void * ignore) {
  tray.tare_position();
  trayTarget = 10;

  while(true) {
    int power = trayTarget - tray.get_position();
    tray.move(power * 0.5);

    delay(25);
  }
}

void releaseCubes() {
  trayTarget = 10;
  double start = millis();
  while(tray.get_position() > 50 && millis() - start < 3000) {}
}

void stackCubes() {
  trayTarget = 600;
}

void armControl(void * ignore) {
  arm.tare_position();

  while(true) {
    arm.move(armTarget - arm.get_position());

    delay(25);
  }
}

void lowTower() {
  trayTarget = 250;
  armTarget = LOW_TOWER_RIGHT;
}

void highTower() {
  trayTarget = 250;
  armTarget = HIGH_TOWER_HEIGHT;
}

void armDown() {
  trayTarget = 10;
  armTarget = 0;
}
