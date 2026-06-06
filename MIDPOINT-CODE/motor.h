#ifndef MOTOR_H
#define MOTOR_H

void motor_init();
void motor_coast();
void motor_driveLR(float uL, float uR);
long motor_getEncL();
long motor_getEncR();

#endif 
