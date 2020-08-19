#ifndef MODEL_H
#define MODEL_H

enum Leg {
	FrontLeft = 0,
	BackLeft = 3,
	FrontRight = 6,
	BackRight = 9 
};

enum MotorPosition {
	Angle = 0,
	Height = 1,
	Radius = 2
};

unsigned get_servo_index(Leg leg, MotorPosition position);




#endif // !MODEL_H




