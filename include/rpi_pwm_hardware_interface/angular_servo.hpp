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

#pragma once

class Servo {
    public:
        Servo(int pi, int pin);
        int getPulseWidth();
        void setPulseWidth(int pulseWidth);

        int __pi;  // TODO(anyone) rename to indicate public, and pigpio pi

    protected:
        int __pin;
};

class AngularServo : public Servo {
    public:
        AngularServo(int pi, int pin, float minAngle, float maxAngle, int minPulseWidthUs,
                    int maxPulseWidthUs);
        void setAngle(float angle);
        int getAngle();

    protected:
        float __angle;
        float __minAngle;
        float __maxAngle;
        int __minPulseWidthUs;
        int __maxPulseWidthUs;
};

bool isPigpiodRunning();
void killPigpiod();
void startPigpiod();
