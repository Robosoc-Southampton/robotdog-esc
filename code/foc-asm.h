#ifndef _FOC_ASM_H
#define _FOC_ASM_H

#include <stdfix.h>
#include <stdint.h>
#include <p33CK64MC103.h>

#define asm_func __attribute__((save(CORCON)))

/* Update a PID loop, returning its output value.
 * constants - kp, ki, kd
 * minmax - lower output bound, upper output bound
 * curr_err - current error
 * prev_err - previous error
 * integral - the accumulator value (treat as opaque, init to 0) */
extern _Fract pid_update(_Fract constants[3], _Fract minmax[2],
			 _Fract curr_err, _Fract *prev_err,
			 uint16_t integral[3]) __attribute__((save(CORCON)));

extern _Fract asm_func foc_update(_Fract ia, _Fract ib, _Fract sin_theta,
			 _Fract cos_theta);

extern void asm_func velocity_pid_update(void);

extern _Fract id, iq, curr_velocity, target_velocity;
extern _Fract target_q, target_d;
extern uint16_t pwm_ta, pwm_tb, pwm_tc;
extern _Fract vd,vq;

#endif /* _FOC_ASM_H */
