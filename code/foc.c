//#include <stdfix.h>
#include <ctype.h>
#include <stdint.h>
#include "foc-asm.h"

#pragma config IESO = OFF
#pragma config FNOSC = FRCPLL

#define MAX(a,b) ((a > b) ? a : b)
#define MIN(a,b) ((a < b) ? a : b)

#define BOUNDARY_BETWEEN_ZONES(a, b) (((a-b)==5 || (b-a)==5) ? 0 : MAX(a,b))

#define HALL_TIMER_MAX 10000000

#define ANGLE_30 2000
#define ANGLE_60 4000
#define ANGLE_90 6000
#define ANGLE_360 24000

#define BEMF_EPSILON 100

#define PWM_PERIOD 8000

#define TIMER_FREQ 100000000

/* The times from foc_update are 15 bit unsigned integers.
 * This defines the right-shift to scale this to a valid period/duty. */
#define PWM_RIGHT_SHIFT 2

enum state_type {
     STATE_OFF,
     STATE_CALIBRATE_ADC,
     STATE_RUNNING,
} state_t;

extern _Fract sin_lookup_tbl[ANGLE_90 >> 4];

/* kp, ki, kd */
_Fract velocity_pid_consts[3] = {0.3r, 0.01r, 0r};
/* Shifts to apply to P, I, D, and result (positive = right shift) */
int16_t vel_pid_shifts[4] = {0, 5, 0, -4};
_Fract pid_current_limits[2] = {-0.15r, 0.15r};

_Fract current_pid_consts[3] = {0.01r, 0.001r, 0r};
_Fract pid_voltage_limits[2] = {-0.5r, 0.5r};
int16_t current_pid_shifts[4] = {0, 0, 0, -8};

uint16_t phase_current_offset[2][8] = {{0}, {0}};
uint8_t phase_current_index[2] = {0, 0};
uint16_t phase_current_avg[2] = {0, 0};
uint32_t phase_current_total[2] = {0, 0};

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

void config_adc(void)
{
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

typedef enum PWM_Output_Typedef {
     PWM_OUT_A, PWM_OUT_B, PWM_OUT_C
} PWM_Output_t;

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

inline uint32_t read_tmr(void)
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

inline int8_t boundary_step(uint8_t prev, uint8_t new)
{
     if (new - prev == 5) return -1;
     if (prev - new == 5) return 1;
     return new - prev;
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
     
     INTCON2bits.GIE = 1;
     IEC6 |= (1 << 8);

     vq = -0.5r;
     vd = 0;

     target_q = 0.2r;
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

     tmp = pwm_output_is_high(PWM_OUT_A);
     count++;

     /* Wait for conversion to complete */
     if (!ADSTATLbits.AN13RDY) {
	  count2++;
	  return;
     }
     
     if (PG1STAT & 2) {
	  do_calibrate_adcs();
     } else {
	  switch (state) {
	  case STATE_OFF:
	       state = STATE_CALIBRATE_ADC;
	       break;
	  case STATE_CALIBRATE_ADC:
	       if (phase_current_index[1] == 7) {
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
	       
	       sin_theta = fract_sin(theta);
	       cos_theta = fract_cos(theta);

	       target_velocity = 0.01r;
	       vel = hall_get_velocity() >> 10;

	       /* 1 rps = 7.15e-4, 1400 rps = 1 */
	       curr_velocity = *(_Fract *)(&vel);
	       
	       /* Determine updated PWM */
	       foc_update(ia, ib, sin_theta, cos_theta);
	       
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


     
