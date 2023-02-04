#ifndef _I2C_H
#define _I2C_H

/* Addresses 48-63 */
#define I2C_ADDR_BASE 0x30

typedef enum I2C_State_Typedef {
     I2C_IDLE,
     I2C_WRITING_ADDR, I2C_WRITING_REG,
     I2C_READING_REG,
     I2C_READING_SAMPLES,
} I2C_State_t;

typedef enum Channel_Typdef {
     CHAN_ID = 0, CHAN_IQ,
     CHAN_THETA, CHAN_VEL,
     CHAN_VD, CHAN_VQ,
} Channel_t;

void config_i2c(void);
void i2c_set_addr(uint8_t addr);
void i2c_state_step(void);

/* Insert data point into queue. This function will figure out
   whether to store the data point or not, depending on the
   sampling configuration */
void push_data_point(Channel_t channel, uint16_t data);
void begin_sample(void);

static inline void push_data_point_fp(Channel_t channel, _Fract data)
{
     push_data_point(channel, *(uint16_t *)(&data));
}

#endif /* _I2c_H */
