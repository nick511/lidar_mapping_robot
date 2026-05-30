#ifndef MOTOR_HELPERS_H
#define MOTOR_HELPERS_H

void motorSetup();
void encoderSetup();
void enableMotors();
void disableMotors();

void setMotorByVel(float linear_x, float angular_z, float delta_time);

#endif
