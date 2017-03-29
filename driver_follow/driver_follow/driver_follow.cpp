/***************************************************************************

                 file : driver_cruise.cpp
    description : user module for CyberFollow

 ***************************************************************************/

/*      
     WARNING !
     
	 DO NOT MODIFY CODES BELOW!
*/

#ifdef _WIN32
#include <windows.h>
#endif

#include "driver_follow.h"


static void userDriverGetParam(float LeaderXY[2], float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_follow(tModInfo *modInfo)
{
	memset(modInfo, 0, 10*sizeof(tModInfo));
	modInfo[0].name    = "driver_follow";	// name of the module (short).
	modInfo[0].desc    =  "user module for CyberFollower" ;	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId    = 0;
	modInfo[0].index   = 0;
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void *pt)
{
	tUserItf *itf = (tUserItf *)pt;
	itf->userDriverGetParam = userDriverGetParam;
	itf->userDriverSetParam = userDriverSetParam;
	return 0;
}

/*
     WARNING!

	 DO NOT MODIFY CODES ABOVE!
*/

/* 
    define your variables here.
    following are just examples
*/
static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm;
static int _gearbox;
static float _Leader_X, _Leader_Y;

static void userDriverGetParam(float LeaderXY[2], float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm){
	/* write your own code here */
	
	_Leader_X = LeaderXY[0];
	_Leader_Y = LeaderXY[1];
	for (int i = 0; i< 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;

	/* you can modify the print code here to show what you want */
	printf("speed %.3f Leader XY(%.3f, %.3f)\n", _speed, _Leader_X, _Leader_Y);
}

static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear){
	/* write your own code here */

	if (_Leader_X*_Leader_X + _Leader_Y*_Leader_Y < 2000) {
		*cmdAcc = 0;
		*cmdBrake = 1;
		//*cmdSteer = _yaw * 1.5;
		*cmdSteer = (_yaw -8*atan2( _midline[30][0],_midline[30][1]))/3.14 ;
		*cmdGear = 1;
	} else {
		*cmdAcc = 0.6;
		*cmdBrake = 0;
		*cmdSteer = (_yaw -8*atan2( _midline[30][0],_midline[30][1]))/3.14 ;
		*cmdGear = 1;
	}
}
