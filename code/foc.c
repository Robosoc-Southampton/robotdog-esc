//#include <stdfix.h>
#include <ctype.h>
#include <stdint.h>
#include "foc-asm.h"
#include "adc.h"
#include "hall.h"
#include "pwm.h"
#include "i2c.h"
#include "utils.h"

#pragma config IESO = OFF
#pragma config FNOSC = FRCPLL
#pragma config ICS = PGD2
#pragma config ALTI2C1 = OFF

enum state_type {
     STATE_OFF,
     STATE_SET_I2C_ADDR,
     STATE_I2C_TEST,
     STATE_CALIBRATE_ADC,
     STATE_RUNNING,
} state_t;

/* kp, ki, kd */
//_Fract velocity_pid_consts[3] = {0.3r, 0.01r, 0r};
/* Shifts to apply to P, I, D, and result (positive = right shift) */
//int16_t vel_pid_shifts[4] = {0, 5, 0, -4};
//_Fract pid_current_limits[2] = {-0.15r, 0.15r};

_Fract current_pid_consts[3] = {0.01r, 0.001r, 0r};
_Fract pid_voltage_limits[2] = {-0.5r, 0.5r};
int16_t current_pid_shifts[4] = {0, 0, 0, -8};

_Fract state_fb_k1 = 0r, state_fb_k1_shift = 0r;
_Fract state_fb_k2 = 0r, state_fb_k2_shift = 0r;
_Fract state_fb_k3 = 0r, state_fb_k3_shift = 0r;
_Fract alpha = 0r, alpha_shift = 0r;

_Fract position = 0r, target_pos = 0r; 

void config_osc(void)
{
     PLLFBDbits.PLLFBDIV = 100;
     PLLDIVbits.POST1DIV = 2;
     __builtin_write_OSCCONH(0x01); /* FRCPLL */
     __builtin_write_OSCCONL(0x01); /* Initiate clock switch */

     while (OSCCONbits.OSWEN)
	  ;

     while (!OSCCONbits.LOCK)
	  ;
}

void config_io(void)
{
     TRISB |= 0b1111; /* GPIO */
     CNPUB |= 0b1111; /* Pull-ups on GPIOs */
}

/* void config_qei(void) */
/* { */
/*      QEI1CONbits.QEIEN = 1; /\* Enable *\/ */
/*      QEI1CONbits.INTDIV = 0; /\* No prescaler *\/ */
/*      QEI1CONbits.CCM = 0; /\* Quadrature Encoder mode *\/ */
/* } */

uint16_t point = 0;

uint16_t pwms[128][3];
uint16_t theta = 0;

int main(void)
{
     /* Config stuff */
     config_osc();
     config_pwm();
     config_adc();
     config_hall();
     config_io();
     config_i2c();
     
     INTCON2bits.GIE = 1;
     IEC6 |= (1 << 8);

     vq = 0.2r;
     vd = 0;

     //target_q = 0.01r;
     target_d = 0r;

     while(1);
     
     return 0;
}

/* Called when the measurement for the phase B current starts */
void _ISR _ADCAN13Interrupt(void)
{
     _Fract ia = 0, ib = 0, sin_theta = 0, cos_theta = 0;
     uint16_t theta = 0;
     static uint16_t state = STATE_OFF;
     static uint32_t count = 0, count2 = 0;
     int16_t vel = 0;
     uint8_t tmp = 0;

     /* tmp = pwm_output_is_high(PWM_OUT_A); */
     /* count++; */

     i2c_state_step();
     
     /* Wait for conversion to complete */
     if (!ADSTATLbits.AN13RDY) {
	  count2++;
	  return;
     }

     begin_sample();
     
     if (PG1STAT & 2) {
	  /* PWM "on-cycle" - all phases high */
	  do_calibrate_adcs();
     } else {
	  /* PWM "off-cycle" - all phases low */
	  switch (state) {
	  case STATE_OFF:
	       state = STATE_SET_I2C_ADDR;
	       break;
	       
	  case STATE_SET_I2C_ADDR:
	       i2c_set_addr(I2C_ADDR_BASE);// + (PORTB & 0b1111));
	       state = STATE_I2C_TEST;
	       state = STATE_CALIBRATE_ADC;
	       break;

	  case STATE_I2C_TEST:
	       i2c_state_step();
	       break;
	       
	  case STATE_CALIBRATE_ADC:
	       if (adc_calibration_done()) {
		    state = STATE_RUNNING;
	       }
	       break;
	       
	  case STATE_RUNNING:
	       /* 1.0 = 20A, -1.0 = -20A */
	       /* 0.05 = 1A, 0.01 = 200mA */
	       ia = read_ia();
	       ib = read_ib();
	       
	       /* Calculate sin and cos of rotor angle */
	       theta = hall_get_angle();
	       //theta = 4000;
	       push_data_point(CHAN_THETA, theta);
	       
	       sin_theta = fract_sin(theta);
	       cos_theta = fract_cos(theta);

	       vel = hall_get_velocity() >> 10;
	       push_data_point(CHAN_VEL, vel);
	       
	       /* 1 rps = 7.15e-4, 1400 rps = 1 */
	       curr_velocity = *(_Fract *)(&vel);
	       
	       /* Determine updated PWM */
	       foc_update(ia, ib, sin_theta, cos_theta);

	       /* Push current/voltage data */
	       push_data_point_fp(CHAN_ID, id);
	       push_data_point_fp(CHAN_IQ, iq);
	       push_data_point_fp(CHAN_VD, vd);
	       push_data_point_fp(CHAN_VQ, vq);
	       
	       /* Update PWM */
	       set_pwm_by_times(pwm_ta, pwm_tb, pwm_tc);
	       
	       break;
	  }
     }
     IFS6 &= ~(1 << 8);
}


/* /\** Ramp mode */
/*  * - Return angle equal to timer value */
/*  *\/ */

/* #define RAMP_RATIO 100 */
/* #define RAMP_MAX (ANGLE_360 * (uint32_t)RAMP_RATIO) */

/* void config_ramp(void) */
/* { */
/*      /\* Setup CCP1 as 32 bit timer *\/ */
/*      CCP1PRL = RAMP_MAX & 0xFFFF; */
/*      CCP1PRH = RAMP_MAX >> 16; */
/*      CCP1CON1Lbits.MOD = 0; */
/*      CCP1CON1Lbits.T32 = 1; */
/*      CCP1CON1Lbits.CLKSEL = 0; /\* Peripheral clock = Fosc/2 = 100MHz *\/ */
/*      CCP1CON1Lbits.CCPON = 1; */
/* } */

/* void set_ramp_angle(uint16_t angle) */
/* { */
/*      CCP1TMRL = angle & 0xFFFF; */
/*      CCP1TMRH = (uint32_t)angle >> 16; */
/* } */

/* uint16_t ramp_get_angle(void) */
/* { */
/*      uint16_t tmr_h, tmr_l; */
/*      uint32_t tmr; */
     
/*      /\* Get (consistent) timer *\/ */
/*      do { */
/* 	  tmr_h = CCP1TMRH; */
/* 	  tmr_l = CCP1TMRL; */
/*      } while (tmr_h != CCP1TMRH); */

/*      tmr = ((uint32_t)tmr_h << 16) + tmr_l; */
/*      return tmr / RAMP_MAX; */
/* } */

/* /\** Determining rotor angle in rotary encoder mode */
/*  * - QEI module provides count and pulse timer */
/*  * - Last period length used to determine speed */
/*  * - Angle calculated using count, speed and time since last period */
/*  *\/ */

/* int16_t hall_angle_prev, hall_angle_curr; /\* Previous and current angles *\/ */
/* uint32_t hall_timer; /\* Time between them *\/ */

/* /\* Expected order (forwards): */
/*  * A on, B off, C on, A off, B on, C off. */
/*  * A transition between two adjacent states indicates that the rotor is */
/*  * at a particular angle. Only the first state-change in a row at a particular */
/*  * angle is recorded to avoid errors due to noise. */
/*  *\/ */
/* void hall_ioc_irq(void) */
/* { */
/*      uint16_t tmr_l, tmr_h, port, change_pin, change_bitno, pos; */

/*      /\* Get pin values *\/ */
/*      port = (PORTC >> 3) & 0b111; */
/*      change_pin = port ^ prev_state; */

/*      if (!change_pin) return; /\* No change *\/ */
     
/*      /\* Get (consistent) timer *\/ */
/*      do { */
/* 	  tmr_h = CCP1TMRH; */
/* 	  tmr_l = CCP1TMRL; */
/*      } while (tmr_h != CCP1TMRH); */
     
/*      /\* Check for invalid port and change_pin *\/ */
/*      if ((port == 0) || (port == 0b111) || */
/* 	 ((change_pin != 1) && (change_pin != 2) && (change_pin != 4))) { */
/* 	  prev_state = port; */
/* 	  return; */
/*      } */

/*      /\* Get bit number of first 1 from right *\/ */
/*      asm("ff1r %1, %0" : "=r"(change_bitno) : "r"(change_pin)); */
/*      change_bitno -= 1; */
     
/*      port = (port << 3) + port; /\* Repeat bits *\/ */

/*      /\* If the pin after the change pin is a 1, it's 180 degrees ahead *\/ */
/*      pos = ((port & (change_pin << 1)) ? 3 : 0) + change_bitno; */

/*      if (pos == hall_angle_curr) return; /\* No change *\/ */

/*      /\* Update angles and timer *\/ */
/*      hall_angle_prev = hall_angle_curr; */
/*      hall_angle_curr = pos; */
/*      hall_timer = ((uint32_t)tmr_h << 16) + tmr_l; */

/*      CCP1TMRL = 0; */
/*      CCP1TMRH = 0; */
/* } */

/* uint16_t hall_get_angle(void) */
/* { */
/*      int16_t step; */
/*      uint16_t tmr_h, tmr_l, prev_angle; */
/*      int32_t tmr, angle; */
/*      uint32_t prev_tmr; */
     
/*      /\* Get direction of last change *\/ */
/*      step = hall_angle_curr - hall_angle_prev; */
/*      if (step == 5) step = -1; */

/*      prev_angle = hall_angle_curr; */
/*      prev_tmr = hall_timer; */
     
/*      /\* Get (consistent) timer *\/ */
/*      do { */
/* 	  tmr_h = CCP1TMRH; */
/* 	  tmr_l = CCP1TMRL; */
/*      } while (tmr_h != CCP1TMRH); */

/*      if (tmr_h > 4) tmr_h = 4; /\* Ensure multiplication won't overflow *\/ */
/*      tmr = ((uint32_t)tmr_h << 16) + tmr_l; */

/*      /\* Calculate the angle relative to the previous edge *\/ */
/*      if (tmr < prev_tmr) { */
/* 	  angle = (tmr * ANGLE_60) / prev_tmr; */
/*      } else if (tmr < ((prev_tmr * 3) / 2)) { */
/* 	  angle = 2*ANGLE_60 - (tmr * ANGLE_60) / prev_tmr; */
/*      } else { */
/* 	  angle = ANGLE_60/2; */
/*      } */

/*      /\* Either add or subtract to previous angle *\/ */
/*      angle = ANGLE_60 * prev_angle + (step > 0 ? angle : -angle); */

/*      while (angle >= ANGLE_360) angle -= ANGLE_360; */
/*      while (angle < 0) angle += ANGLE_360; */

/*      return (uint16_t)angle; */
/* } */

/* void config_rot(void) */
/* { */

/* } */

/* void rot_qei_irq(void) */
/* { */

/* } */

/* uint16_t rot_get_angle(void) */
/* { */
/*      return 0; */
/* } */

/* /\** Determining rotor angle in back-emf mode (startup) */
/*  * - Assume rotor angle to be increasing according to a ramp */
/*  * - Regulate phase current to predefined value */
/*  * */
/*  * Determining rotor angle in back-emf mode (running) */
/*  * - Use previous estimate to determine when one of the phases will cross 0 */
/*  * - Turn off that phase slightly before */
/*  * - Measure phase voltages until they reach 0 */
/*  * - Record time of zero-crossing, and determine speed */
/*  *\/ */
/* void config_bemf(void) */
/* { */
/*      /\* RA3=A, RA4=B, RC2=C *\/ */
/* } */

/* uint16_t bemf_get_angle(void); */

/* enum bemf_mode { BEMF_RAMP, BEMF_CALIBRATE, BEMF_CALIBRATE_FLOAT, */
/*      BEMF_NORMAL, BEMF_NORMAL_FLOAT }; */
/* enum bemf_mode bemf_mode = BEMF_RAMP; */

/* uint16_t bemf_times[2]; */
/* uint16_t bemf_angles[2]; /\* Same order as hall mode *\/ */

/* /\* Buffer of 4 results for averaging *\/ */
/* uint16_t bemf_adc_buf[4]; */

/* uint16_t bemf_dir; /\* 0 for increasing angle, 1 for decreasing *\/ */

/* /\* If a zero-crossing is coming up, make it float, and periodically */
/*  * measure back-emf. If the back-emf is already past the crossing, */
/*  * measure the rate of change of the back-emf and extrapolate. Use the */
/*  * rate of change to determine the angular velocity, and use this to */
/*  * determine the acceleration (using the time between the previous */
/*  * zero-crossings to determine previous velocity). */
/*  * */
/*  * In order to calculate the angular velocity from the rate of change */
/*  * of back-emf, the voltage constant must be known. This will be */
/*  * obtained in BEMF_CALIBRATE mode, which will use the times between */
/*  * zero-crossings to find a zero-crossing of constant velocity, and */
/*  * then divide the velocity by the angular frequency. *\/ */
/* void bemf_update(void) */
/* { */
/*      uint16_t angle, remainder, delta; */
     
/*      switch (bemf_mode) { */
/*      case BEMF_RAMP: */
/* 	  return; */
/*      case BEMF_CALIBRATE: */
/*      case BEMF_NORMAL: */
/* 	  /\* - Check current angle */
/* 	   * - If angle is within epsilon of zero-crossing, float phase */
/* 	   *\/ */
/* 	  angle = bemf_get_angle(); */
/* 	  remainder = angle % ANGLE_60; */

/* 	  delta = bemf_dir ? remainder : ANGLE_60 - remainder; */

/* 	  if (delta < BEMF_EPSILON) { */
/* 	       // Float phase */
	       
/* 	       bemf_mode = bemf_mode == BEMF_NORMAL ? BEMF_NORMAL_FLOAT : */
/* 		    BEMF_CALIBRATE_FLOAT; */
/* 	  } */
/* 	  break; */
/*      case BEMF_CALIBRATE_FLOAT: */
/* 	  /\* - Measure all three phase voltages */
/* 	   * - Calculate back-emf */
/* 	   * - Store in buffer */
/* 	   * - Find average */
/* 	   * - If average has crossed 0, determine gradient and calculate constant */
/* 	   * - If the zero-crossing is detected before enough measurements can */
/* 	   *   be taken, continue taking measurement, else re-enable the phase. */
/* 	   *\/ */
/* 	  break; */
/*      case BEMF_NORMAL_FLOAT: */
/* 	  /\* - Measure phase voltages */
/* 	   * - Calculate back-emf */
/* 	   * - Store in buffer */
/* 	   * - Find average */
/* 	   * - If average has crossed 0, determine gradient and calculate constant */
/* 	   *\/ */
/* 	  break; */
/*      default: */
/* 	  break; */
/*      } */
/* } */

/* uint16_t bemf_get_angle(void) */
/* { */
/*      return 0; */
/* } */


     
