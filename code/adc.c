#include <p33CK64MC103.h>
#include "adc.h"

uint16_t phase_current_offset[2][8] = {{0}, {0}};
uint8_t phase_current_index[2] = {0, 0};
uint16_t phase_current_avg[2] = {0, 0};
uint32_t phase_current_total[2] = {0, 0};

void config_adc(void)
{
     ANSELA = 0;
     ANSELB = 0;
     
     ANSELC |= 0b11;
     TRISC |= 0b11;
     
     ADCON1Hbits.FORM = 1; /* Fractional */
     ADCON1Hbits.SHRRES = 0b11; /* 12 bit resolution */
     ADCON3Lbits.REFSEL = 0; /* AVdd */
     ADCON3Hbits.CLKSEL = 0b10; /* Fvco / 3   (267MHz) */
     ADCON3Hbits.CLKDIV = 1; /* Clock divided by 2 (133MHz) */
     ADCON2Lbits.SHRADCS = 0; /* Clock divided by 2 (67MHz) */
     ADCON2Hbits.SHRSAMC = 18; /* 0.3us = 20 clk periods */

     /* Phase A - channel 12, phase B - channel 13 */
     ADMOD0Hbits.SIGN12 = 0; /* Unsigned */
     ADMOD0Hbits.SIGN13 = 0; /* Unsigned */
     
     ADCON5Hbits.WARMTIME = 0b1010; /* 1024 clock periods = 15us */
     ADCON1Lbits.ADON = 1;
     ADCON5Lbits.SHRPWR = 1;
     while (!ADCON5Lbits.SHRRDY)
	  ;
     ADCON3Hbits.SHREN = 1;
     
     ADTRIG3Lbits.TRGSRC12 = 0b00100; /* PWM1 trigger 1 */
     ADTRIG3Lbits.TRGSRC13 = 0b00110; /* PWM2 trigger 1 */

     ADIEL |= (1 << 13); /* Enable interrupt */
}

void do_calibrate_adcs(void)
{
     uint8_t ind, i;
     uint16_t prev_offset, reading;

     for (i=0; i < 2; i++) {
	  ind = phase_current_index[i];
	  prev_offset = phase_current_offset[i][ind];
	  reading = (i == 0) ? ADCBUF12 : ADCBUF13;

	  phase_current_offset[i][ind] = reading;
	  phase_current_total[i] += reading;
	  phase_current_total[i] -= prev_offset;

	  phase_current_avg[i] = phase_current_total[i] >> 3;

	  if (++ind == 8) ind = 0;
	  
	  phase_current_index[i] = ind;
     }
}

uint8_t adc_calibration_done(void)
{
     return phase_current_index[1] == 7;
}

/* 1m shunt: Circuit gain = 0.001V/1A * 27.5 * 65536/3.3V = 546 counts per 1A.
             29.3mA resolution.
 * 3m shunt: Circuit gain = 0.003V/1A * 27.5 * 65536/3.3V = 1638 counts per 1A.
             9.8mA resolution. */
_Fract read_ia(void)
{
     uint16_t adc;
     int16_t x;
     _Fract *ptr;

     adc = ADSTATLbits.AN12RDY ? ADCBUF12 : 0;
     
     /* Subtract offset from ADC, ensuring no overflow during conversion,
      * then cast back to int16_t */
     x = (int16_t)((int32_t)adc - phase_current_avg[0]);
     ptr = (_Fract *)(&x);
     return *ptr;
}

_Fract read_ib(void)
{
     uint16_t adc;
     int16_t x;
     _Fract *ptr;

     adc = ADSTATLbits.AN13RDY ? ADCBUF13 : 0;
     
     /* Subtract offset from ADC, ensuring no overflow during conversion,
      * then cast back to int16_t */
     x = (int16_t)((int32_t)adc - phase_current_avg[1]);
     ptr = (_Fract *)(&x);
     return *ptr;
}
