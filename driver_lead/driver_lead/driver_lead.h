/***************************************************************************

    file                 : user1.h
    author            : Xuangui Huang
    email              : stslxg@gmail.com
    description    :  leader module for CyberFollow

 ***************************************************************************/

#include "tgf.h"

#ifndef __LEADER_ITF
#define __LEADER_ITF

/* CyberFollow Leader Interface */
typedef void (*tfudLeaderGetParam) (float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm, float DistanceFromStart, int laps);
typedef void (*tfudLeaderSetParam) (float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);

typedef  struct {
	tfudLeaderGetParam userDriverGetParam;
	tfudLeaderSetParam userDriverSetParam;
} tLeaderItf;

#endif