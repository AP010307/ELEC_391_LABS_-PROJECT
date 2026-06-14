#ifndef ANGLE_H
#define ANGLE_H

bool  angle_init();
void  angle_calibrate();
float angle_update(float dt);
float angle_get();

#endif 
