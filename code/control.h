#ifndef _CONTROL_H
#define _CONTROL_H

#include <stdint.h>

void controller_step_time(void);
uint16_t get_target_pos(void);

/* Ptr to statically allocated array containing target: position,
   velocity, acceleration, jerk */
int16_t *get_target_derivs(void);

/* Buffer each coefficient as it arrives. When the transmission
   ends, flush the coefficients, and it will do the right thing.
   If flush_coeffs is called when no coefficients have been written,
   do nothing */
void buffer_coeff(uint8_t num, uint16_t val);
void flush_coeffs(void);

void set_waveform_scale(uint16_t val);
void set_waveform_delay(uint16_t val);

#endif /* _CONTROL_H */
