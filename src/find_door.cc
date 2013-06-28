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
#define MAX_SHIFT 0.3
#define MAX_DOOR_WIDTH 1.2
#define MIN_DOOR_WIDTH 0.5
#define PI 3.14159265

double door_width(const RangerProxy& rp, int left, int right) {
  double la = rp.GetMinAngle() + left * rp.GetAngularRes();
  double lx = rp[left] * cos(la);
  double ly = rp[left] * sin(la);

  double ra = rp.GetMinAngle() + right * rp.GetAngularRes();
  double rx = rp[right] * cos(ra);
  double ry = rp[right] * sin(ra);

  double dx = rx - lx;
  double dy = ry - ly;
  return sqrt(dx * dx + dy * dy);
}

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
    if (left < 0 && grad[i] > GRAD_THRESHOLD) {
      left = i - 1;
      if (left < 0) {
        left = 0;
      }
    }
    else if (left >= 0 && grad[i] < -GRAD_THRESHOLD 
        && scan[i] - scan[left] < MAX_SHIFT) {
      right = i + 1;
      if (right >= size) {
        right = size - 1;
      }
      // Filter
      double dw = door_width(scan, left, right);
      if (dw > MAX_DOOR_WIDTH || dw < MIN_DOOR_WIDTH) {
        left = -1;
        right = -1;
      }
    }
  }

  delete grad;

  if (left >= 0 && right >= 0) {
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

double sign(double x) {
  if (x > 0) {
    return 1.0;
  }
  else if (x < 0) {
    return -1.0;
  }
  else {
    return 0.0;
  }
}

int main(int argc, char** argv) {
  PlayerClient    robot("localhost");
  RangerProxy     rp(&robot, 1);
  Position2dProxy pp(&robot, 0);

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
      double rx = rp[right] * cos(ra);
      double ry = rp[right] * sin(ra);

      printf("Door found at angle %f~%f\n", la, ra);
      printf("Left (%f, %f)\n", lx, ly);
      printf("Right (%f, %f)\n", rx, ry);
      printf("Door width %f\n", door_width(rp, left, right));

      if (fabs((la + ra) / 2.0) < .1) {
        printf("Straight -- %f\n", fabs((la + ra) / 2.0));
        printf("Turn\n");
        pp.SetSpeed(0.2, 0, 0);
      }
      else {
        printf("Turn\n");
        pp.SetSpeed(0, 0, .3 * sign(la + ra));
      }
    }
    else {
      printf("No Door detected\n");
      pp.SetSpeed(0, 0, .3);
    }
  }
}
