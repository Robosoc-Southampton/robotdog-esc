#ifndef _HALL_H
#define _HALL_H

#include <stdint.h>
#include <p33CK64MC103.h>
#include "utils.h"

#define BOUNDARY_BETWEEN_ZONES(a, b) (((a-b)==5 || (b-a)==5) ? 0 : MAX(a,b))

#define HALL_TIMER_MAX 10000000

void config_hall(void);

static inline uint32_t read_tmr(void)
{
     uint16_t tmr_h, tmr_l;

     do {
	  tmr_h = CCP1TMRH;
	  tmr_l = CCP1TMRL;
     } while (tmr_h != CCP1TMRH);

     /* Don't let the timer overflow */
     if (tmr_h > 16000) {
	  tmr_h = 16000;
	  CCP1TMRH = 16000;
     }
     
     return ((uint32_t)tmr_h << 16) + tmr_l;
}

static inline int8_t boundary_step(uint8_t prev, uint8_t new)
{
     if (new - prev == 5) return -1;
     if (prev - new == 5) return 1;
     return new - prev;
}

uint16_t hall_get_angle(void);
int32_t hall_get_velocity(void);

#endif /* _HALL_H */
