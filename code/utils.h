#ifndef _UTILS_H
#define _UTILS_H

#define MAX(a,b) ((a > b) ? a : b)
#define MIN(a,b) ((a < b) ? a : b)

#define ANGLE_30 2000
#define ANGLE_60 4000
#define ANGLE_90 6000
#define ANGLE_360 24000

#define TIMER_FREQ 100000000

extern _Fract sin_lookup_tbl[ANGLE_90 >> 4];

static inline _Fract fract_sin(uint16_t theta)
{
     while (theta >= ANGLE_360) theta -= ANGLE_360;
     
     if (theta < ANGLE_90) {
	  return sin_lookup_tbl[theta >> 4];
     } else if (theta < 2*ANGLE_90) {
	  return sin_lookup_tbl[(ANGLE_90*2 - 1 - theta) >> 4];
     } else if (theta < 3*ANGLE_90) {
	  return -sin_lookup_tbl[(theta - 2*ANGLE_90) >> 4];
     } else {
	  return -sin_lookup_tbl[(ANGLE_90*4 - 1 - theta) >> 4];
     }
}

static inline _Fract fract_cos(uint16_t theta)
{
     return fract_sin(theta + ANGLE_90);
}

#endif /* _UTILS_H */
