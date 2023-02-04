#ifndef _ADC_H
#define _ADC_H

#include <stdfix.h>
#include <stdint.h>

void config_adc(void);
void do_calibrate_adcs(void);
uint8_t adc_calibration_done(void);
_Fract read_ia(void);
_Fract read_ib(void);

#endif /* _ADC_H */
