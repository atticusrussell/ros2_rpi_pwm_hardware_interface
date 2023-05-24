#pragma once

class Servo {
	public:
		Servo(int pi, int pin);
		int getPulseWidth();
		void setPulseWidth(int pulseWidth);

		int __pi; // TODO rename to indicate public, and pigpio pi
	protected:
		int __pin;
};

class AngularServo : public Servo {
	public:
		AngularServo(int pi, int pin, float minAngle, float maxAngle, int minPulseWidthUs, int maxPulseWidthUs);
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

