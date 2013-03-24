#ifndef __H_ACCELERATORS_H__
#define __H_ACCELERATORS_H__
#include "area_accelerator.h"

#define USE_ACCEL_AREA	1 << 1
#define USE_ACCEL_COLOR	1 << 2
#define USE_ACCEL_AND	1 << 3
#define USE_ACCEL_ALL	USE_ACCEL_AREA | USE_ACCEL_COLOR | USE_ACCEL_AND
#define USE_ACCEL_NONE	0

#endif
