#include <p33CK64MC103.h>
#include "pwm.h"

void config_pwm(void)
{
     /* PWM1 triggers current measurement for phase A, PWM2 triggers
      * current measurement for phase B. Phase A is measured near the
      * maximum timer point and phase B is measured near the bottom,
      * both during the time when all three phases are low. Both
      * require a postscaler of 2, so that they are triggered at the
      * right times. */

     TRISB &= ~(0b111111 << 10);
     
     PCLKCON = 0b01; /* No divider, master clock source is Fvco/2 */
     MPER = 7999; /* Period of 20us */
     
     /* Master clock, center-aligned */
     PG1CONL = 0b0000000000001100;
     PG2CONL = 0b0000000000001100;
     PG3CONL = 0b0000000000001100;

     /* Local registers except PER, trigger and start of
      * cycle. Trigger is local for PG1 and PG1 for the others. Master
      * update enabled for PG1 */
     PG1CONH = 0b0100100000000000; /* SOC update */
     PG2CONH = 0b0100001000000001; /* Client SOC update */
     PG3CONH = 0b0100001000000001; /* Client SOC update */
     
     /* Enable pins, H active high, L active low */
     PG1IOCONH = 0b1101;
     PG2IOCONH = 0b1101;
     PG3IOCONH = 0b1101;

     /* ADC trigger + postscaler, manual update requests, EOC as trigger */
     PG1EVTL = 0b0000001100000000;//0b0000001100000000;
     PG2EVTL = 0b0000001100000000;//0b0000001100000000;
     PG3EVTL = 0b0000000000000000;

     /* Interrupts, ADC trigger 2, trigger 1 offset */
     PG1EVTH = 0b0000001100000000;
     PG2EVTH = 0b0000001100000000;
     PG3EVTH = 0b0000001100000000;

     /* Duty cycle = 50% (min 8, max 7992) */
     PG1DC = 4000;
     PG2DC = 4000;
     PG3DC = 4000;

     /* Trigger A at 7800 and 199 for 0.5us either side */
     PG1TRIGA = 7800 | (1 << 15); /* Second half */
     PG2TRIGA = 50;//199 | (0 << 15); /* First half */

     /* Trigger B same as A, but for when PWM is high */
     PG1TRIGB = 7800 | (0 << 15); /* First half */
     PG2TRIGB = 50 | (1 << 15); /* Second half */
     
     /* Dead-time of 1us */
     PG1DTH = 400;
     PG1DTL = 400;
     PG2DTH = 400;
     PG2DTL = 400;
     PG3DTH = 400;
     PG3DTL = 400;

     /* Enable generators */
     PG1CONL |= 0x8000;
     PG2CONL |= 0x8000;
     PG3CONL |= 0x8000;
}

uint8_t pwm_output_is_high(PWM_Output_t op) {
     switch (op)
     {
     case PWM_OUT_A:
	  return !!(PORTB & (1 << 15));
     case PWM_OUT_B:
	  return !!(PORTB & (1 << 13));
     case PWM_OUT_C:
	  return !!(PORTB & (1 << 11));
     default:
	  return 0;
     }
}

void set_pwms(uint16_t pwm_a, uint16_t pwm_b, uint16_t pwm_c)
{
     PG1DC = pwm_a;
     PG2DC = pwm_b;
     PG3DC = pwm_c;
     PG1STAT |= (1 << 3);
     PG2STAT |= (1 << 3);
     PG3STAT |= (1 << 3);
}

void set_pwm_by_times(uint16_t ta, uint16_t tb, uint16_t tc)
{
     /* tmax is the greatest duty, before d0 is added. d0 is
      * the duty to be added to all three phases. */
     uint16_t tmax, d0;

     ta = ta >> PWM_RIGHT_SHIFT;
     tb = tb >> PWM_RIGHT_SHIFT;
     tc = tc >> PWM_RIGHT_SHIFT;
     
     tmax = MAX(MAX(ta, tb), tc);

     /* Period - 2*d0 = tmax, since when d0 is added to tmax,
      * the remaining off-time is equal to d0. */
     d0 = (PWM_PERIOD - tmax) >> 1;

     /* d0 must be >= 8 for correct function of PWM */
     if (d0 < 8) d0 = 8;

     set_pwms(ta+d0, tb+d0, tc+d0);
}
