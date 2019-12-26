#include "main.h"
#include "mech_lib.hpp"
#include "base_lib.hpp"
#include "driver_funcs.hpp"

void releaseCubes() {
  intake(-80);
  delay(50);
  base(-127, -127);
  delay(200);
  intake(0);
  base(0, 0);

  lowerTray();
}
