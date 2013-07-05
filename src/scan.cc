#include <iostream>
#include <libplayerc++/playerc++.h>

using namespace PlayerCc;

int main(int argc, char** argv) {
  PlayerClient    robot("localhost");
  RangerProxy     scan(&robot, 1);

  robot.Read();
  
  int size = scan.GetRangeCount();

  printf("%d %f %f\n", size, scan.GetMinAngle(), scan.GetAngularRes());
  for (int i = 0; i < size; i++) {
    printf("%f\n", scan[i]);
  }
}

