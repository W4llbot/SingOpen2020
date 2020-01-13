#include "main.h"
#include "autons.hpp"
#include "base_lib.hpp"
#include "mech_lib.hpp"

void redBack() {
  resetCoord(0, 0, 0);

  intake(127);
  baseMove(0, 44, 0.35, 0.09);
  waitBase(2);
  baseMove(0, 40, 0.35, 0.09);
  waitBase(1);

  baseTurn(-8, 56, 1, 0);
  waitBase(2);
  // baseMove(-7, 42, 0.35, 0.09);
  // waitBase(1);
  //
  // delay(500);
  // baseMove(0, 44);
  // waitBase(1);
}
