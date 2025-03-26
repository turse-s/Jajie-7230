#ifndef _TaskTimer_H
#define _TaskTimer_H
#include "parameters.h"
double HotPursuit(double goal,double now,double step,int dir);
int Up_Or_Down(double angle1, double angle2);
void Task_Timer(void *p);
#endif
