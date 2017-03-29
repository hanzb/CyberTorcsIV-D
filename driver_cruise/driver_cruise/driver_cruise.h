#include "tgf.h"

#ifndef __USER_ITF
#define __USER_ITF

#include<cmath>

/* CyberCruise User Interface */
typedef void (*tfudGetParam) (float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
typedef void (*tfudSetParam) (float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);

typedef  struct {
	tfudGetParam userDriverGetParam;
	tfudSetParam userDriverSetParam;
} tUserItf;

#endif
