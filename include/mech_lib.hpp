#ifndef _MECH_LIB_HPP
#define _MECH_LIB_HPP

void trayControl(void * ignore);
void armControl(void * ignore);

void intake(int speed);

void releaseCubes();
void stackCubes();

void highTower();
void lowTower();
void armDown();

#endif
