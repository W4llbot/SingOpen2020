#include "main.h"
#include "base_lib.hpp"

Motor FL(FLPort);
Motor BL(BLPort);
Motor FR(FRPort);
Motor BR(BRPort);

void base(int left, int right) {
  FL.move(left);
  BL.move(left);
  FR.move(right);
  BR.move(right);
}
