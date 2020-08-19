#include "model.h"

unsigned get_servo_index(Leg leg, MotorPosition position) {
	return ((unsigned)leg) + (unsigned(position));
}

