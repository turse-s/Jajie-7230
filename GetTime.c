#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "GetTime.h"

long int get_time(void)
{
 
	time_t t;
	t = time(NULL);
 
	int ii = time(&t);

	return ii;

}