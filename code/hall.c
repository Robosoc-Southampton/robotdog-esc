#include "hall.h"

/** Determining rotor angle in hall sensor mode
 * - Interrupt on change of any hall pin
 * - Store last two changes and measure time between them
 * - Use time to calculate speed, and estimate rotor angle
 */

//uint16_t prev_state = 0; /* Bit 0 = A */

void config_hall(void)
{
     /* RC3=HALL_A, RC4=HALL_B, RC5=HALL_C */
     TRISC |= 0b111000;
     ANSELC &= ~0b111000;
     /* CNCONCbits.ON = 1; */
     /* CNCONCbits.CNSTYLE = 1; /\* Edge notification *\/ */
     /* CNEN0C |= 0b111000; */
     /* CNEN1C |= 0b111000; */

     /* /\* Initialise prev_state *\/ */
     /* uint16_t x = PORTC; */
     /* prev_state = ((x >> 3) & 0b111); */
     /* if ((prev_state == 0) || (prev_state == 0b111)) { */
     /* 	  // Error */
     /* } */

     /* Set up CCP1 as 32 bit timer */
     CCP1PRL = 0xFFFF;
     CCP1PRH = 0xFFFF;
     CCP1CON1Lbits.MOD = 0;
     CCP1CON1Lbits.T32 = 1;
     CCP1CON1Lbits.CLKSEL = 0; /* Peripheral clock = Fosc/2 = 100MHz */
     CCP1CON1Lbits.CCPON = 1;
}

uint8_t hall_states[6] = {0b011, 0b010, 0b110, 0b100, 0b101, 0b001};
//uint8_t hall_states[6] = {0b010, 0b011, 0b001, 0b101, 0b100, 0b110};
uint8_t hall_state_offset = 0;

uint16_t hall_prev_boundary = 0, hall_curr_boundary = 0;
uint16_t hall_curr_zone = 0;
int32_t hall_velocity = 0; /* Measured as angle per second (1rps = 24000) */
uint32_t hall_prev_tmr = 0;
//int8_t hall_direction = 1;

uint32_t hall_tmr_buffer[8] = {0};
uint8_t hall_tmr_index = 0;
uint64_t hall_tmr_total = 0;

uint16_t hall_get_angle(void)
{
     uint16_t port, _hallzone, hallzone, boundary;
     int16_t hallangle;
     uint32_t tmr, tmp;
     int16_t step;

     port = (PORTC >> 3) & 0b111;

     for (_hallzone = 0; _hallzone < 6; _hallzone++) {
	  hallzone = (_hallzone + hall_state_offset) >= 6 ?
	       (_hallzone + hall_state_offset) - 6 : _hallzone + hall_state_offset;
	  if (port == hall_states[hallzone]) break;
     }

     tmr = read_tmr();
     
     /* Measured zone is different to last recorded */
     if (hallzone != hall_curr_zone) {
	  /* Determine the boundary between the two most recent zones */
	  boundary = BOUNDARY_BETWEEN_ZONES(hallzone, hall_curr_zone);

	  /* If the boundary differs from the last read boundary */
	  if (hall_curr_boundary != boundary) {
	       /* Update boundary readings */
	       hall_prev_boundary = hall_curr_boundary;
	       hall_curr_boundary = boundary;

	       hall_prev_tmr = tmr;

	       /* Update hall timer buffer */
	       tmp = hall_tmr_buffer[hall_tmr_index];
	       hall_tmr_buffer[hall_tmr_index] = tmr;
	       hall_tmr_total -= tmp; /* Subtract old reading */
	       hall_tmr_total += tmr; /* Add new reading */

	       if (++hall_tmr_index == 8) hall_tmr_index = 0;

	       /* Reset timer */
	       CCP1TMRL = 0;
	       CCP1TMRH = 0;
	  }

	  hall_curr_zone = hallzone;
     }

     /* Get direction of last change */
     step = boundary_step(hall_prev_boundary, hall_curr_boundary);
     
     /* Calculate angle relative to the previous edge */
     if (hall_prev_tmr == 0 || hall_prev_tmr > HALL_TIMER_MAX) {
	  hallangle = ANGLE_60 * hall_curr_zone;
     } else {
	  if (tmr < hall_prev_tmr) {
	       hallangle = ((uint64_t)tmr * ANGLE_60) / hall_prev_tmr;
	       hallangle = ANGLE_60 * hall_curr_boundary - ANGLE_30 +
		    (step > 0 ? hallangle : -hallangle);
	  } else if (tmr < ((hall_prev_tmr * 3) / 2)) {
	       hallangle = 2*ANGLE_60 - ((uint64_t)tmr * ANGLE_60) / hall_prev_tmr;
	       hallangle = ANGLE_60 * hall_curr_boundary - ANGLE_30 +
		    (step > 0 ? hallangle : -hallangle);
	  } else {
	       hallangle = ANGLE_60 * hall_curr_zone;
	  }
     }

     hallangle -= 0;//4000;
     while (hallangle >= ANGLE_360) hallangle -= ANGLE_360;
     while (hallangle < 0) hallangle += ANGLE_360;

     return hallangle;
}



int32_t hall_get_velocity(void)
{
     int8_t dir;
     uint32_t t;

     dir = boundary_step(hall_prev_boundary, hall_curr_boundary);

     t = read_tmr();
     t = MAX(t, hall_tmr_total >> 3);
     
     if (hall_prev_tmr < 1000) {
	  return 0;
     } else {
	  return dir * (((int64_t)TIMER_FREQ * ANGLE_60) / t);
     }
}
