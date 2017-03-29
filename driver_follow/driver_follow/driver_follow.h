#include "tgf.h"

#ifndef __USER2_ITF
#define __USER2_ITF

/* CyberCruise User Interface */
typedef void (*tfudGetParam) (float LeaderXY[2], float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
typedef void (*tfudSetParam) (float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);

typedef  struct {
	tfudGetParam userDriverGetParam;
	tfudSetParam userDriverSetParam;
} tUserItf;

#endif