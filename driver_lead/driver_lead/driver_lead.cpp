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

#include "driver_lead.h"
#include <cmath>
static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm, float DistanceFromStart, int laps);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

typedef struct Circle									//
{														//
	double r;											//
	int sign;											//
}circle;
circle getR(float x1, float y1, float x2, float y2, float x3, float y3);
void updateGear(int *cmdGear);
double constrain(double lowerBoundary, double upperBoundary, double input);

// Module Entry Point
extern "C" int driver_lead(tModInfo *modInfo)
{
	memset(modInfo, 0, 10*sizeof(tModInfo));
	modInfo[0].name    = "driver_lead";	// name of the module (short).
	modInfo[0].desc    =  "leader module for CyberFollow" ;	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId    = 0;
	modInfo[0].index   = 0;
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void *pt)
{
	tLeaderItf *itf = (tLeaderItf *)pt;
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
static float _yaw, _yawrate, _speed, _acc, _width, _rpm,  _DistanceFromStart;
static int _gearbox, _laps;
circle c;
const int topGear = 6;
int flag = 0,j=0;
static double err = 6.7;

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm,  float DistanceFromStart, int laps){
	/* write your own code here */
	
	for (int i = 0; i< 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
	_DistanceFromStart = DistanceFromStart;
	_laps = laps;

	printf("speed %f DFS %f lap %d 10m far target(%f, %f)\n", _speed, _DistanceFromStart, _laps, _midline[10][0], _midline[10][1]);
	
}

static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear){
	/* write your own code here */
	/*******************************路径设计********************************/
	c = getR(_midline[30][0], _midline[30][1], _midline[40][0], _midline[40][1], _midline[50][0], _midline[50][1]);
	switch(_laps)
	{
		case 0:
			*cmdGear = 1;//档位始终挂1
			printf_s("gear:%d\n", *cmdGear);
			*cmdBrake = 0;//无刹车
			*cmdAcc = 0.8;
			*cmdSteer = (_yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / 3.14;//设定舵机方向
			//初始保护
			break;
	    case 1: 
			*cmdSteer = (_yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / 3.14;
			*cmdBrake = 0;
			//*cmdAcc = 0.2;
			//*cmdGear = 1;
			if(_speed <= 70)
			*cmdAcc = -0.006*_speed+1 ;
			else *cmdAcc = 0.4;
			if (c.r >= 1000)
			{
				if (*cmdGear < 6) {
					*cmdGear += 1;
				}
			}
			if (*cmdGear == 6 && _speed <= 200)
				*cmdAcc = 2;
			else if (_speed > 140) {
				*cmdBrake = 0.5;
				updateGear(cmdGear);
			}
			break;//第一圈加入高档低速
	    case 2: 

			if (c.r >= 1000) {
				flag = 1;
				*cmdAcc = 0.5;
				updateGear(cmdGear);
				printf("%lf", _midline[30][0]);
				++j;
				if (j == 102) {
					 *cmdSteer = (_yaw - 8 * atan2(_midline[30][0] + err, _midline[30][1])) / 3.14;
					j = 0;
					err = -1*err;
				}
				else {
					if (abs(_yaw) < 0.005){
						*cmdSteer = 0;
					}
					else {
						*cmdSteer = (_yaw - 8 * atan2(_midline[30][0] + err, _midline[30][1])) / 3.14;
					}
				}
			}//直道：高速左右迂回（利用路宽）

			else {

				if (flag) {
					if (_speed >= 160) {
						*cmdBrake = 1;
						*cmdAcc = 0;
						*cmdGear = 4;
					}
					else if(_speed<=120){ 
						*cmdBrake = 0; 
						*cmdAcc = 1;
						*cmdGear = 4;
					}
					*cmdSteer = (_yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / 3.14;
				}//接下来的弯道循中线,速度在140~180间波动。

				else {
					if ((abs(_yaw) <= 0.004) && (c.r > 90) && abs(_midline[15][0]) <= 7.5) {
						*cmdSteer = 0;
					}
					else 
						*cmdSteer = (_yaw - 8 * atan2(_midline[30][0] + 7.5, _midline[30][1])) / 3.14;
					if (_speed >= 187) {
						*cmdBrake = 0.3;
						*cmdAcc = 0;
						updateGear(cmdGear);
					}
					else {
						*cmdBrake = 0;
						*cmdAcc = 0.8;
						updateGear(cmdGear);
					}
					}
			}//弯道：先走外圈
			break; //第二圈最后减速
		case 3: 	*cmdAcc = 0;//无油门
			*cmdBrake = 0.1;//刹车
			*cmdSteer = (_yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / 3.14;//设定舵机方向
			updateGear(cmdGear);
			//第三圈的时候进行一定的保护
	    default:break;
	}
	printf("lap %d \n",_laps);
	
}

circle getR(float x1, float y1, float x2, float y2, float x3, float y3)
{
	double a, b, c, d, e, f;
	double r, x, y;

	a = 2 * (x2 - x1);
	b = 2 * (y2 - y1);
	c = x2*x2 + y2*y2 - x1*x1 - y1*y1;
	d = 2 * (x3 - x2);
	e = 2 * (y3 - y2);
	f = x3*x3 + y3*y3 - x2*x2 - y2*y2;
	x = (b*f - e*c) / (b*d - e*a);
	y = (d*c - a*f) / (b*d - e*a);
	r = sqrt((x - x1)*(x - x1) + (y - y1)*(y - y1));
	x = constrain(-1000.0, 1000.0, x);
	y = constrain(-1000.0, 1000.0, y);
	r = constrain(1.0, 5000.0, r);
	int sign = (x>0) ? 1 : -1;
	circle tmp = { r,sign };
	return tmp;
}

void updateGear(int *cmdGear)
{
	if (_gearbox == 1)
	{
		if (_speed >= 60 && topGear >1)
		{
			*cmdGear = 2;
		}
		else
		{
			*cmdGear = 1;
		}
	}
	else if (_gearbox == 2)
	{
		if (_speed <= 45)
		{
			*cmdGear = 1;
		}
		else if (_speed >= 105 && topGear >2)
		{
			*cmdGear = 3;
		}
		else
		{
			*cmdGear = 2;
		}
	}
	else if (_gearbox == 3)
	{
		if (_speed <= 90)
		{
			*cmdGear = 2;
		}
		else if (_speed >= 145 && topGear >3)
		{
			*cmdGear = 4;
		}
		else
		{
			*cmdGear = 3;
		}
	}
	else if (_gearbox == 4)
	{
		if (_speed <= 131)
		{
			*cmdGear = 3;
		}
		else if (_speed >= 187 && topGear >4)
		{
			*cmdGear = 5;
		}
		else
		{
			*cmdGear = 4;
		}
	}
	else if (_gearbox == 5)
	{
		if (_speed <= 173)
		{
			*cmdGear = 4;
		}
		else if (_speed >= 234 && topGear >5)
		{
			*cmdGear = 6;
		}
		else
		{
			*cmdGear = 5;
		}
	}
	else if (_gearbox == 6)
	{
		if (_speed <= 219)
		{
			*cmdGear = 5;
		}
		else
		{
			*cmdGear = 6;
		}
	}
	else
	{
		*cmdGear = 1;
	}
}

double constrain(double lowerBoundary, double upperBoundary, double input)
{
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
	else
		return input;
}