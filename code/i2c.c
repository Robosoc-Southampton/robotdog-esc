#include <p33CK64MC103.h>
#include "i2c.h"

#define MSG_RECEIVED I2C1STATbits.RBF
#define WRITE_ISSUED !I2C1STATbits.R_W
#define READ_ISSUED I2C1STATbits.R_W
#define SLAVE_INT_FLAG IFS1bits.SI2C1IF
#define ADDRESS_MSG !I2C1STATbits.D_A
#define DATA_MSG I2C1STATbits.D_A
#define TX_BUF_EMPTY !I2C1STATbits.TBF
#define STOP_ISSUED I2C1STATbits.P

#define REG_CONSTANT(n) (0x00 + n)
#define REG_SAMPLE_VALUES 0x10
#define REG_SAMPLE_RATE_DIV 0x11
#define REG_WAVEFORM_SCALE 0x12
#define REG_WAVEFORM_DELAY 0x13
#define REG_WAVEFORM_C(n) (0x14 + n)
#define REG_BUFFER_AVAILABLE 0x18
#define REG_SAMPLE_BUFFER 0x19

#define REG_NUM 32

#define CHANNEL_BIT(c) (1 << c)

#define SAMPLE_BUF_SIZE 2048
#define SAMPLE_BUF_MASK (SAMPLE_BUF_SIZE - 1)
#define SAMPLE_BUF_OVERFLOWN(r) (!!(r[REG_BUFFER_AVAILABLE] & 0x80))
#define SAMPLE_BUF_SET_OVERFLOWN(r, v) r[REG_BUFFER_AVAILABLE] = \
	  v ? (r[REG_BUFFER_AVAILABLE] | 0x80) : (r[REG_BUFFER_AVAILABLE] & ~0x80)

static uint16_t i2c_registers[REG_NUM];

static uint16_t sample_buffer[SAMPLE_BUF_SIZE];
static uint16_t buffer_start = 0, buffer_len = 0, buffer_of = 0;
static uint16_t sample_div_counter = 0;

void config_i2c(void)
{
     /* Enable; 7-bit addr; slew control enabled; disable general call */
     I2C1CONL = 0b1001000101000000;

     I2C1CONH = 0b000011;
}

void i2c_set_addr(uint8_t addr)
{
     I2C1ADD = addr;
}

static void i2c_process_data(int8_t reg, uint8_t data)
{
     static uint8_t curr_reg = 0, bytenum = 0, buffer = 0;

     if (reg != -1) /* New register selected */
     {
	  curr_reg = reg;
	  bytenum = 0;
     }

     if (bytenum == 0) /* Low byte written (buffer it) */
     {
	  buffer = data;
	  bytenum = 1;
     }
     else /* High byte written (store both low and high byte together) */
     {
	  i2c_registers[curr_reg] = (data << 8) + buffer;
	  bytenum = 0;

	  switch (curr_reg)
	  {
	  case REG_SAMPLE_RATE_DIV:
	       sample_div_counter = 0;
	       break;
	  case REG_WAVEFORM_SCALE:
	       set_waveform_scale(i2c_registers[curr_reg]);
	       break;
	  case REG_WAVEFORM_DELAY:
	       set_waveform_delay(i2c_registers[curr_reg]);
	       break;
	  case REG_WAVEFORM_C(0):
	       buffer_coeff(0, i2c_registers[curr_reg]);
	       break;
	  case REG_WAVEFORM_C(1):
	       buffer_coeff(1, i2c_registers[curr_reg]);
	       break;
	  case REG_WAVEFORM_C(2):
	       buffer_coeff(2, i2c_registers[curr_reg]);
	       break;
	  case REG_WAVEFORM_C(3):
	       buffer_coeff(3, i2c_registers[curr_reg]);
	       break;
	  case REG_BUFFER_AVAILABLE:
	       if (i2c_registers[REG_BUFFER_AVAILABLE] == 0)
	       {
		    buffer_len = 0;
		    buffer_of = 0;
	       }
	       break;
	  }

	  curr_reg++;
     }
}

static uint8_t i2c_read_data(int8_t reg)
{
     static uint8_t curr_reg = 0, bytenum = 0, buffer = 0;
     uint8_t retval = 0;
     
     if (reg != -1)
     {
	  curr_reg = reg;
	  bytenum = 0;
     }

     if (bytenum == 0) /* Low byte being read, buffer high byte */
     {
	  retval = i2c_registers[curr_reg] & 0xFF;
	  buffer = i2c_registers[curr_reg] >> 8;
	  bytenum = 1;
     }
     else
     {
	  retval = buffer;
	  bytenum = 0;
	  curr_reg++;
     }
     return retval;
}

static uint8_t i2c_read_sample(int8_t reg)
{
     static uint8_t bytenum = 0;
     uint8_t retval = 0;
     
     if (reg != -1)
     {
	  bytenum = 0;
     }

     if (bytenum == 0)
     {
	  retval = sample_buffer[buffer_start] & 0xFF;
	  bytenum = 1;
     }
     else
     {
	  retval = sample_buffer[buffer_start] >> 8;
	  buffer_start = (buffer_start + 1) & SAMPLE_BUF_MASK;
	  buffer_len--;

	  if (buffer_len == 0)
	  {
	       buffer_of = 0;
	  }
	  bytenum = 0;
     }

     i2c_registers[REG_BUFFER_AVAILABLE] = buffer_len | (buffer_of << 15);
     return retval;
}

void begin_sample(void)
{
     if (sample_div_counter == i2c_registers[REG_SAMPLE_RATE_DIV])
     {
	  sample_div_counter = 0;
     }
     else
     {
	  sample_div_counter++;
     }
}	  

void push_data_point(Channel_t channel, uint16_t data)
{
     uint16_t index;

     if (sample_div_counter == 0)
     {
	  if (CHANNEL_BIT(channel) & i2c_registers[REG_SAMPLE_VALUES])
	  {
	       if (buffer_len == SAMPLE_BUF_SIZE)
	       {
		    buffer_of = 1;
	       }
	       
	       if (!buffer_of)
	       {
		    index = (buffer_start + buffer_len) & SAMPLE_BUF_MASK;
		    sample_buffer[index] = data;
		    buffer_len++;
	       }
	  }
     }

     i2c_registers[REG_BUFFER_AVAILABLE] = buffer_len | (buffer_of << 15);
}

static I2C_State_t i2c_state = I2C_IDLE;

void i2c_state_step(void)
{
     static uint8_t data = 0;
     static int8_t reg_addr = 0;

     /* If a stop bit has been sent, or an address has been received,
	transmission has been stopped/restarted. Go back to idle. */
     if (STOP_ISSUED ||
	 (MSG_RECEIVED && ADDRESS_MSG))
     {
	  i2c_state = I2C_IDLE;

	  /* In case the previous transmission included coefficients,
	     flush them so the controller knows the transmission has
	     finished */
	  flush_coeffs();
     }
     
     switch (i2c_state)
     {
     case I2C_IDLE:
	  if (MSG_RECEIVED)
	  {
	       if (WRITE_ISSUED) /* Write command received */
	       {
		    i2c_state = I2C_WRITING_ADDR;
	       }
	       else if (READ_ISSUED) /* Read command received */
	       {
		    if (reg_addr == REG_SAMPLE_BUFFER) /* Current register is sample read */
		    {
			 i2c_state = I2C_READING_SAMPLES;
		    }
		    else
		    {
			 i2c_state = I2C_READING_REG;
		    }
	       }
	       data = I2C1RCV; /* Read register to clear flag */
	       I2C1CONLbits.SCLREL = 1;
	  }
	  break;
	  
     case I2C_WRITING_ADDR:
	  if (MSG_RECEIVED && DATA_MSG) /* Register address received */
	  {
	       reg_addr = I2C1RCV;
	       i2c_state = I2C_WRITING_REG;
	       I2C1CONLbits.SCLREL = 1;
	  }
	  break;
	  
     case I2C_WRITING_REG:
	  if (MSG_RECEIVED && DATA_MSG) /* Data received */
	  {
	       data = I2C1RCV;
	       i2c_process_data(reg_addr, data);
	       reg_addr = -1; /* -1 indicates that i2c_process/read_data should
				 determine address based on its own count */
	       I2C1CONLbits.SCLREL = 1;
	  }
	  break;
	  
     case I2C_READING_REG:
	  if (TX_BUF_EMPTY) /* Buffer empty */
	  {
	       /* Load buffer */
	       I2C1TRN = i2c_read_data(reg_addr);
	       reg_addr = -1;
	       I2C1CONLbits.SCLREL = 1;
	  }
	  break;
	  
     case I2C_READING_SAMPLES:
	  if (TX_BUF_EMPTY) /* Buffer empty */
	  {
	       /* Load buffer */
	       I2C1TRN = i2c_read_sample(reg_addr);
	       reg_addr = -1;
	       I2C1CONLbits.SCLREL = 1;
	  }
	  break;
     }
}
