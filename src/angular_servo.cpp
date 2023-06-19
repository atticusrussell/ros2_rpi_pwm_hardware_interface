// Copyright (c) 2023, Atticus Russell
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <pigpiod_if2.h>

#include <csignal>
#include <cstdlib>
#include <functional>
#include <iostream>

#include "rpi_pwm_hardware_interface/angular_servo.hpp"

using std::cout, std::cerr, std::endl;

// TODO(anyone) revisit private vs protected vars - maybe swap to private

Servo::Servo(int pi, int pin) {
  __pi = pi;
  __pin = pin;
  int rc;
  rc = set_mode(__pi, __pin, PI_OUTPUT);
  if (rc < 0) {
    throw "Invalid GPIO pin or mode Error!";
  }

  // initializing to zero with pigpio is no input
  rc = set_servo_pulsewidth(__pi, __pin, 0);
  if (rc < 0) {
    throw "Invalid user GPIO pin or pulsewidth Error!";
  }
}

int Servo::getPulseWidth() {
  int rc = get_servo_pulsewidth(__pi, __pin);
  if (rc < 0) {
    throw "Invalid user GPIO pin Error!";
  }
  return rc;
}

void Servo::setPulseWidth(int pulseWidth) {
  int rc = set_servo_pulsewidth(__pi, __pin, pulseWidth);
  if (rc < 0) {
    throw "Invalid user GPIO pin or pulsewidth Error!";
  }
}

AngularServo::AngularServo(int pi, int pin, float minAngle, float maxAngle, int minPulseWidthUs,
    int maxPulseWidthUs) :
Servo(pi, pin) {
  __minAngle = minAngle;
  __maxAngle = maxAngle;
  __minPulseWidthUs = minPulseWidthUs;
  __maxPulseWidthUs = maxPulseWidthUs;
}

void AngularServo::setAngle(float angle) {
  __angle = angle;
  // make sure angle is within bounds
  if (__angle < __minAngle) {
    __angle = __minAngle;
  }
  if (__angle > __maxAngle) {
    __angle = __maxAngle;
  }
  int pulseWidth = __minPulseWidthUs +
      (__angle - __minAngle) * (__maxPulseWidthUs - __minPulseWidthUs) / (__maxAngle - __minAngle);

  setPulseWidth(pulseWidth);
}

int AngularServo::getAngle() {
  int pulseWidth = getPulseWidth();
  float angle = __minAngle +
      (pulseWidth - __minPulseWidthUs) * (__maxAngle - __minAngle) /
          (__maxPulseWidthUs - __minPulseWidthUs);
  return angle;
}

bool isPigpiodRunning() {
  int result = system("pgrep pigpiod");

  if (result == 0) {
    // pigpiod daemon is running
    return true;
  } else {
    // pigpiod daemon is not running
    return false;
  }
}

void killPigpiod() {
  if (isPigpiodRunning()) {
    int result = system("sudo killall pigpiod -q");

    if (result == 0) {
      // Successfully killed the pigpiod daemon
      cout << "pigpiod daemon killed successfully" << endl;
    } else {
      // An error occurred while trying to kill the pigpiod daemon
      cerr << "Error killing pigpiod daemon. Return code: " << result << endl;
    }
  } else {
    cout << "pigpiod daemon is not running" << endl;
  }
}

void startPigpiod() {
  if (!isPigpiodRunning()) {
    cout << "pigpio daemon not running. attempting to start it." << endl;
    int rc = system("sudo systemctl start pigpiod");
    sleep(1);

    if (isPigpiodRunning() && (rc != -1)) {
      // Successfully started the pigpiod daemon
      cout << "pigpiod daemon started successfully" << endl;
    } else {
      // An error occurred while trying to start the pigpiod daemon
      cerr << "Error starting pigpiod daemon." << endl;
    }
  } else {
    cout << "pigpiod daemon is already running" << endl;
  }
}
