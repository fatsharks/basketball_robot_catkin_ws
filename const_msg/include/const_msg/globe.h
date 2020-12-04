#ifndef _GLOBE_H_
#define _GLOBE_H_

#include <math.h>
#include "const.h"

typedef struct tagPosrture
{
    float theta;
    float x;
    float y;
}Posture;


/*typedef struct tagSYSTEMTIME
{
	int 	system;	//动作运行时间（单位10ms 自动累加多次运动也累加）
	int 	control;//上次控制周期动作运行时间
	int		t;//动作时间间隔＝每次动作控制时的时间－上次控制周期动作运行时间
}TIMESTRUCT;*/

#endif
