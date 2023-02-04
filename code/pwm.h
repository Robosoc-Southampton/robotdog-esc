#ifndef _PWM_H
#define _PWM_H

#include <stdint.h>
#include "utils.h"

#define PWM_PERIOD 8000

/* The times from foc_update are 15 bit unsigned integers.
 * This defines the right-shift to scale this to a valid period/duty. */
#define PWM_RIGHT_SHIFT 2

typedef enum PWM_Output_Typedef {
     PWM_OUT_A, PWM_OUT_B, PWM_OUT_C
} PWM_Output_t;

void config_pwm(void);
uint8_t pwm_output_is_high(PWM_Output_t op);
void set_pwms(uint16_t pwm_a, uint16_t pwm_b, uint16_t pwm_c);
void set_pwm_by_times(uint16_t ta, uint16_t tb, uint16_t tc);

#endif /* _PWM_H */
