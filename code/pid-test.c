//#include <stdfix.h>
#include <ctype.h>
#include <stdint.h>
#include "foc-asm.h"

#pragma config IESO = OFF
#pragma config FNOSC = FRCPLL

#define ANGLE_60 4000
#define ANGLE_360 24000

#define BEMF_EPSILON 100

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

/* void config_adc(void) */
/* { */
/*      ANSELC |= 0b11; */
/*      TRISC |= 0b11; */
     
/*      ADCON1Hbits.FORM = 1; /\* Fractional *\/ */
/*      ADCON1Hbits.SHRRES = 0b11; /\* 12 bit resolution *\/ */
/*      ADCON3Lbits.REFSEL = 0; /\* AVdd *\/ */
/*      ADCON3Hbits.CLKSEL = 0b10; /\* Fvco / 3   (267MHz) *\/ */
/*      ADCON3Hbits.CLKDIV = 0; */
/*      ADCON2Lbits.SHRADCS = 0; /\* Clock divided by 2 (133MHz) *\/ */
/*      ADCON2Hbits.SHRSAMC = 0; /\* Divided by 2 again (67MHz) *\/ */

/*      /\* Phase A - channel 12, phase B - channel 13 *\/ */
/*      ADMOD0Hbits.SIGN12 = 0; /\* Unsigned *\/ */
/*      ADMOD0Hbits.SIGN13 = 0; /\* Unsigned *\/ */
     
/*      ADCON5Hbits.WARMTIME = 0b1010; /\* 1024 clock periods = 15us *\/ */
/*      ADCON1Lbits.ADON = 1; */
/*      ADCON5Lbits.SHRPWR = 1; */
/*      while (!ADCON5Lbits.SHRRDY) */
/* 	  ; */
/*      ADCON3Hbits.SHREN = 1; */
     
/*      ADTRIG3Lbits.TRGSRC12 = 0b00100; /\* PWM1 trigger 1 *\/ */
/*      ADTRIG3Lbits.TRGSRC13 = 0b00110; /\* PWM2 trigger 1 *\/ */
/* } */

/* void config_pwm(void) */
/* { */
/*      /\* PWM1 triggers current measurement for phase A, PWM2 triggers */
/*       * current measurement for phase B. Phase A is measured near the */
/*       * maximum timer point and phase B is measured near the bottom, */
/*       * both during the time when all three phases are low. Both */
/*       * require a postscaler of 2, so that they are triggered at the */
/*       * right times. *\/ */

/*      TRISB &= ~(0b111111 << 10); */
     
/*      PCLKCON = 0b01; /\* No divider, master clock source is Fvco/2 *\/ */
/*      MPER = 7999; /\* Period of 20us *\/ */
     
/*      /\* Master clock, center-aligned *\/ */
/*      PG1CONL = 0b0000000000001100; */
/*      PG2CONL = 0b0000000000001100; */
/*      PG3CONL = 0b0000000000001100; */

/*      /\* Local registers except PER, trigger and start of */
/*       * cycle. Trigger is local for PG1 and PG1 for the others. Master */
/*       * update enabled for PG1 *\/ */
/*      PG1CONH = 0b0100100000000000; /\* SOC update *\/ */
/*      PG2CONH = 0b0100001000000001; /\* Client SOC update *\/ */
/*      PG3CONH = 0b0100001000000001; /\* Client SOC update *\/ */
     
/*      /\* Enable pins, H active high, L active low *\/ */
/*      PG1IOCONH = 0b1101; */
/*      PG2IOCONH = 0b1101; */
/*      PG3IOCONH = 0b1101; */

/*      /\* ADC trigger + postscaler, manual update requests, EOC as trigger *\/ */
/*      PG1EVTL = 0b0000000100000000; */
/*      PG2EVTL = 0b0000000100000000; */
/*      PG3EVTL = 0b0000000000000000; */

/*      /\* Interrupts, ADC trigger 2, trigger 1 offset *\/ */
/*      PG1EVTH = 0b0000001100000000; */
/*      PG2EVTH = 0b0000001000000000; /\* Enable interrupt at ADC trigger 1 *\/ */
/*      PG3EVTH = 0b0000001100000000; */

/*      /\* Duty cycle = 50% (min 8, max 7992) *\/ */
/*      PG1DC = 4000; */
/*      PG2DC = 4000; */
/*      PG3DC = 4000; */

/*      /\* Trigger A at 7800 and 199 for 0.5us either side *\/ */
/*      PG1TRIGA = 7800 | (1 << 15); /\* Second half *\/ */
/*      PG2TRIGA = 199 | (0 << 15); /\* First half *\/ */

/*      /\* Dead-time of 1us *\/ */
/*      PG1DTH = 400; */
/*      PG1DTL = 400; */
/*      PG2DTH = 400; */
/*      PG2DTL = 400; */
/*      PG3DTH = 400; */
/*      PG3DTL = 400; */

/*      /\* Enable generators *\/ */
/*      PG1CONL |= 0x8000; */
/*      PG2CONL |= 0x8000; */
/*      PG3CONL |= 0x8000; */
/* } */

/* void config_qei(void) */
/* { */
/*      QEI1CONbits.QEIEN = 1; /\* Enable *\/ */
/*      QEI1CONbits.INTDIV = 0; /\* No prescaler *\/ */
/*      QEI1CONbits.CCM = 0; /\* Quadrature Encoder mode *\/ */
/* } */

/* void config_spi(void) */
/* { */

/* } */

/* void update_spi(void) */
/* { */

/* } */

/* void set_pwms(uint16_t pwm_a, uint16_t pwm_b, uint16_t pwm_c) */
/* { */

/* } */

uint16_t x[10] = {0x5555, 0x5555};
     
//uint16_t x2 = 0x5555;
int main(void)
{
     _Fract pid_constants[3] = { 0.1r, 0.1r, 0.1r };
     _Fract pid_minmax[2] = { -0.5r, 0.5r };
     uint16_t pid_i[3] = { 0 };
     _Fract out = 0;

     _Fract trials[][4] = {
	  /*{0r, 0r, 0r, 0r},
	  {0.3r, 0.3r, 0r, 0r},
	  {0.3r, 0r, 0r, 0r},
	  {0.3r, 0r, 0r, 0.2r},
	  {0.3r, 0r, 0r, -0.2r},
	  {0.9r, 0.6r, 0r, 0r},
	  {0.9r, 0.6r, 0r, -0.1r},*/
	  {0r, 0r, 0r, 0.6r},
	  {0r, 0r, 0r, -0.6r},
     };

     int t;
     _Fract integral;

     /* Config stuff */
     config_osc();

     while(1)
     {
	  for (t=0; t < 2; t++) {
	       out = pid_update(pid_constants, pid_minmax, trials[t][0],
				&trials[t][1], &trials[t][2]);
	       integral = trials[t][2];
	       out += 0.1r;
	  }
	       
     }
     
     return 0;
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

/* /\** Determining rotor angle in hall sensor mode */
/*  * - Interrupt on change of any hall pin */
/*  * - Store last two changes and measure time between them */
/*  * - Use time to calculate speed, and estimate rotor angle */
/*  *\/ */

/* uint16_t prev_state = 0; /\* Bit 0 = A *\/ */

/* void config_hall(void) */
/* { */
/*      /\* RC3=HALL_A, RC4=HALL_B, RC5=HALL_C *\/ */
/*      TRISC |= 0b111000; */
/*      ANSELC &= ~0b111000; */
/*      CNCONCbits.ON = 1; */
/*      CNCONCbits.CNSTYLE = 1; /\* Edge notification *\/ */
/*      CNEN0C |= 0b111000; */
/*      CNEN1C |= 0b111000; */

/*      /\* Initialise prev_state *\/ */
/*      uint16_t x = PORTC; */
/*      prev_state = ((x >> 3) & 0b111); */
/*      if ((prev_state == 0) || (prev_state == 0b111)) { */
/* 	  // Error */
/*      } */

/*      /\* Set up CCP1 as 32 bit timer *\/ */
/*      CCP1PRL = 0xFFFF; */
/*      CCP1PRH = 0xFFFF; */
/*      CCP1CON1Lbits.MOD = 0; */
/*      CCP1CON1Lbits.T32 = 1; */
/*      CCP1CON1Lbits.CLKSEL = 0; /\* Peripheral clock = Fosc/2 = 100MHz *\/ */
/*      CCP1CON1Lbits.CCPON = 1; */
/* } */

/* int16_t hall_angle_prev, hall_angle_curr; /\* Previous and current angles *\/ */
/* uint32_t hall_timer; /\* Time between them *\/ */

/* /\* Expected order (forwards): */
/*  * A on, B off, C on, A off, B on, C off. */
/*  * A transition between two adjacent states indicates that the rotor is */
/*  * at a particular angle. Only the first state-change in a row at a particular */
/*  * angle is recorded to avoid errors due to noise.  */
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

/* /\** Determining rotor angle in rotary encoder mode */
/*  * - QEI module provides count and pulse timer */
/*  * - Last period length used to determine speed */
/*  * - Angle calculated using count, speed and time since last period */
/*  *\/ */

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


/* /\* Called when the measurement for the phase B current starts *\/ */
/* void adc_irq(void) */
/* { */
/*      _Fract ia = 0, ib = 0, sin_theta = 0, cos_theta = 0; */
     
/*      /\* Wait for conversion to complete *\/ */
/*      while (!ADSTATLbits.AN13RDY) */
/* 	  ; */

/*      /\* Calculate sin and cos of rotor angle *\/ */

/*      /\* Determine updated PWM *\/ */
/*      foc_update(ia, ib, sin_theta, cos_theta); */
     
/*      /\* Update PWM *\/ */
/* } */
     
