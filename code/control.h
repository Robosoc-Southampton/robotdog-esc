#ifndef _CONTROL_H
#define _CONTROL_H



struct motor_control {
     fixedpt16_t rotor_angle;


};

fixedpt16_t get_rotor_angle(adc_t v_a, adc_t v_b, adc_t v_c);

#endif /* _CONTROL_H */
