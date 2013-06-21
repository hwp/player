/*
   Copyright (c) 2005, Brad Kratochvil, Toby Collett, Brian Gerkey, Andrew Howard, ...
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
 this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.
 * Neither the name of the Player Project nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <libplayerc++/playerc++.h>

using namespace PlayerCc;

#define GRAD_THRESHOLD 0.5
#define PI 3.14159265

/** 
 * return number of doors detected (0 || 1).
 */
int find_door(const RangerProxy& scan, int& left, int& right) {
  int size = scan.GetRangeCount();
  left = -1;
  right = -1;

  double* grad = new double[size];
  for (int i = 0; i < size; i++) {
    double l = scan[i > 0 ? i - 1 : 0];
    double r = scan[i + 1 < size ? i + 1 : i];
    grad[i] = (r - l) / 2.0;
  }

  // assume there is only one door
  for (int i = 0; i < size; i++) {
    if (grad[i] > GRAD_THRESHOLD) {
      left = i - 1;
      if (left < 0) {
        left = 0;
      }
    }
    else if (left >= 0 && grad[i] < -GRAD_THRESHOLD) {
      right = i + 1;
      if (right >= size) {
        right = size - 1;
      }
    }
  }

  delete grad;

  if (left >= 0) {
    if (right < 0) {
      right = size - 1;
    }
    return 1;
  }
  else {
    return 0;
  }
}

player_point_2d_t getPoint(const player_pose2d_t& pose) {
  player_point_2d_t r;
  r.px = pose.px;
  r.py = pose.py;
  return r;
}

player_pose2d_t toWorld(const Position2dProxy& pp, double x, double y, double a) {
  player_pose2d_t r;
  r.px = pp.GetXPos() + x * cos(pp.GetYaw()) - y * sin(pp.GetYaw());
  r.py = pp.GetYPos() + x * sin(pp.GetYaw()) + y * cos(pp.GetYaw());
  r.pa = pp.GetYaw() + a;
  return r;
}


int main(int argc, char** argv) {
  PlayerClient    robot("localhost");
  RangerProxy     rp(&robot, 1);
  Position2dProxy pp(&robot, 0);
//Graphics2dProxy gp(&robot);

  /*
     while (1) {
     robot.Read();
     printf("Current Pos (%f, %f, %f)\n", pp.GetXPos(), pp.GetYPos(), pp.GetYaw());
     printf("goto %f, %f, %f\n", 1.0, 0.0, 0.0);
     pp.GoTo(1.0, 0, 0);
     }
   */

  while (1) {
    robot.Read();
    printf("\n\n\n\n\n\n++++++++++++++++++++++\n");
    printf("Current Pos (%f, %f, %f)\n", pp.GetXPos(), pp.GetYPos(), pp.GetYaw());

    int left, right;
    double mx, my, ma;
    if (find_door(rp, left, right)) {
      double la = rp.GetMinAngle() + left * rp.GetAngularRes();
      double lx = rp[left] * cos(la);
      double ly = rp[left] * sin(la);

      double ra = rp.GetMinAngle() + right * rp.GetAngularRes();
      double rx = rp[right] * cos(la);
      double ry = rp[right] * sin(la);

      printf("Door found at angle %f~%f\n", la, ra);
//    player_point_2d_t door[2];
//    door[0] = getPoint(toWorld(pp, lx, ly, 0));
//    door[1] = getPoint(toWorld(pp, rx, ry, 0));
//    gp.DrawMultiline(door, 2);

      if (fabs((la + ra) / 2.0) < .1) {
        printf("Straight -- %f\n", fabs((la + ra) / 2.0));
        mx = 1.0;
        my = mx * (la + ra) / 2.0;
        ma = 0.0;
      }
      else {
        printf("Turn\n");
        mx = 0.0;
        my = 0.0;
        ma = (la + ra) / 2.0;
      }
    }
    else {
      printf("No Door detected\n");
      mx = 0;
      my = 0;
      ma = 1;
    }

    printf("relative(%f, %f, %f)\n", mx, my, ma);
    player_pose2d_t pose = toWorld(pp, mx, my, ma);
    printf("goto(%f, %f, %f)\n", pose.px, pose.py, pose.pa);
    pp.GoTo(pose);
  }
}
