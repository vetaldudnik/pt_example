//******************************************************************************
//
// Copyright (c) 2019, Vitaliy Dudnik <masterv.play@gmail.com>
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//   1. Redistributions of source code must retain the above copyright notice,
//      this list of conditions and the following disclaimer.
//   2. Redistributions in binary form must reproduce the above copyright notice,
//      this list of conditions and the following disclaimer in the documentation
//      and/or other materials provided with the distribution.
//   3. Neither the name of copyright holder nor the names of its contributors
//      may be used to endorse or promote products derived from this software
//      without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//******************************************************************************

#include <system.h>
#include <string.h>
#include <drivers/gpio.h>
#include "i2c.h"

#define SDA_PERIPH                  LL_APB2_GRP1_PERIPH_GPIOB
#define SDA_GPIO                    GPIOB
#define SDA_PIN                     LL_GPIO_PIN_7

#define SCL_PERIPH                  LL_APB2_GRP1_PERIPH_GPIOB
#define SCL_GPIO                    GPIOB
#define SCL_PIN                     LL_GPIO_PIN_6

#define SDA_HIGH                    GPIO_SET(SDA_GPIO, SDA_PIN)
#define SCL_HIGH                    GPIO_SET(SCL_GPIO, SCL_PIN)

#define SDA_LOW                     GPIO_CLR(SDA_GPIO, SDA_PIN)
#define SCL_LOW                     GPIO_CLR(SCL_GPIO, SCL_PIN)

#define SDA_STATE                   GPIO_STATE(SDA_GPIO, SDA_PIN)
#define SCL_STATE                   GPIO_STATE(SCL_GPIO, SCL_PIN)

#define I2C_PARAM_SIZE_MAX          (  4 )
#define I2C_DATA_SIZE_MAX           ( 16 )

typedef enum
{
	I2C_MODE_WRITE,
	I2C_MODE_READ,

} i2c_mode_t;

static u08 i2c_slave_addr;
static u08 i2c_param[I2C_PARAM_SIZE_MAX];
static u08 i2c_param_size;
static u08 i2c_data[I2C_DATA_SIZE_MAX];
static u08 i2c_data_size;
static i2c_mode_t i2c_mode;
static bool i2c_trigger;
static bool i2c_error;
static pt_mutex_t mutex;

static
pt_status_t i2c_start()
{
	PT_ENTER();

	PT_WAIT_FOR(/*SDA_STATE && */SCL_STATE);

	SDA_LOW;

	PT_YIELD();

	PT_EXIT();
}

static
pt_status_t i2c_restart()
{
	PT_ENTER();

	PT_WAIT_FOR(SCL_STATE);

	SCL_LOW;
	SDA_HIGH;

	PT_YIELD();

	SCL_HIGH;

	PT_YIELD();

	PT_WAIT_FOR(SCL_STATE);

	SDA_LOW;

	PT_YIELD();

	PT_EXIT();
}

static
pt_status_t i2c_stop()
{
	PT_ENTER();

	PT_WAIT_FOR(SCL_STATE);

	SCL_LOW;
	SDA_LOW;

	PT_YIELD();

	SCL_HIGH;

	PT_WAIT_FOR(SCL_STATE);

	SDA_HIGH;

	PT_YIELD();

	PT_EXIT();
}

static
pt_status_t i2c_byte_transmitt(u08 byte, u08 *ack)
{
	static u08 mask;

	PT_ENTER();

	for(mask = 0x80; mask; mask >>= 1)
	{
		PT_WAIT_FOR(SCL_STATE);

		SCL_LOW;

		if(byte & mask)
		{
			SDA_HIGH;
		}
		else
		{
			SDA_LOW;
		}

		PT_YIELD();

		SCL_HIGH;

		PT_YIELD();
	}

	SCL_LOW;
	SDA_HIGH;

	PT_YIELD();

	SCL_HIGH;

	PT_YIELD();

	*ack = SDA_STATE ? 0 : 1;

	PT_EXIT();
}

static
pt_status_t i2c_byte_receive(u08 *byte, u08 ack)
{
	static u08 data;
	static u08 mask;

	PT_ENTER();

	data = 0;

	for(mask = 0x80; mask; mask >>= 1)
	{
		PT_WAIT_FOR(SCL_STATE);

		SCL_LOW;
		SDA_HIGH;

		PT_YIELD();

		SCL_HIGH;

		PT_YIELD();

		PT_WAIT_FOR(SCL_STATE);

		if(SDA_STATE)
		{
			data |= mask;
		}
	}

	SCL_LOW;

	if(ack)
	{
		SDA_LOW;
	}
	else
	{
		SDA_HIGH;
	}

	PT_YIELD();

	SCL_HIGH;

	PT_YIELD();

	*byte = data;

	PT_EXIT();
}

static
pt_status_t i2c_proc()
{
	static u08 index;
	       u08 ack;

	PT_ENTER();

	for(;;)
	{
		//
		// Wait for trigger
		//
		PT_WAIT_FOR(i2c_trigger);

		//
		// Generate start condition
		//
		PT_WAIT_FOR(i2c_start());

		//
		// Send slave address + W
		//
		PT_WAIT_FOR(i2c_byte_transmitt(i2c_slave_addr, &ack));

		//
		// Check acknowledge
		//
		if(!ack)
		{
			i2c_error = 1;

			goto complete;
		}

		//
		// Send parameter value
		//
		for(index = 0; index < i2c_param_size; index++)
		{
			PT_WAIT_FOR(i2c_byte_transmitt(i2c_param[index], &ack));

			//
			// Check acknowledge
			//
			if(!ack)
			{
				i2c_error = 1;

				goto complete;
			}
		}

		//
		// Check mode
		//
		switch(i2c_mode)
		{
		case I2C_MODE_READ:
			//
			// Generare repeat start
			//
			PT_WAIT_FOR(i2c_restart());

			//
			// Send slave address + R
			//
			PT_WAIT_FOR(i2c_byte_transmitt(i2c_slave_addr | 0x01, &ack));

			//
			// Check acknowledge
			//
			if(!ack)
			{
				i2c_error = 1;

				goto complete;
			}

			//
			// Receive data from device
			//
			for(index = 0; index < i2c_data_size; index++)
			{
				PT_WAIT_FOR(i2c_byte_receive(&i2c_data[index], index != (i2c_data_size - 1)));
			}

			//
			// Completed successfully
			//
			i2c_error = 0;

			break;

		case I2C_MODE_WRITE:
			//
			// Send data to device
			//
			for(index = 0; index < i2c_data_size; index++)
			{
				PT_WAIT_FOR(i2c_byte_transmitt(i2c_data[index], &ack));

				//
				// Check acknowledge
				//
				if(!ack)
				{
					break;
				}
			}

			//
			// Set error if necessary
			//
			i2c_error = ack ? 0 : 1;

			break;
		}

complete:
		//
		// Generate stop condition
		//
		PT_WAIT_FOR(i2c_stop());

		//
		// Reset trigger
		//
		i2c_trigger = 0;
	}

	PT_EXIT();
}

static
void i2c_begin()
{
	i2c_trigger = 1;
}

static
bool i2c_complete()
{
	return !i2c_trigger;
}

pt_status_t i2c_write(u08 slave_addr, const u08 *param, u08 param_size, const u08 *data, u08 data_size, i2c_status_t *status)
{
	if(!pt_mutex_acquire(&mutex))
	{
		return PT_STATUS_YIELD;
	}

	PT_ENTER();

	if(i2c_param_size <= I2C_PARAM_SIZE_MAX &&
		i2c_data_size <= I2C_DATA_SIZE_MAX)
	{
		i2c_slave_addr = slave_addr;

		memcpy(i2c_param, param, param_size);
		i2c_param_size = param_size;

		memcpy(i2c_data, data, data_size);
		i2c_data_size = data_size;

		i2c_mode = I2C_MODE_WRITE;

		i2c_begin();

		PT_WAIT_FOR(i2c_complete());

		*status = i2c_error ? I2C_STATUS_HW_ERROR : I2C_STATUS_OK;
	}
	else
	{
		*status = I2C_STATUS_VALUE_ERROR;
	}

	pt_mutex_release(&mutex);

	PT_EXIT();
}

pt_status_t i2c_read(u08 slave_addr, const u08 *param, u08 param_size, u08 *data, u08 data_size, i2c_status_t *status)
{
	if(!pt_mutex_acquire(&mutex))
	{
		return PT_STATUS_YIELD;
	}

	PT_ENTER();

	if(i2c_param_size <= I2C_PARAM_SIZE_MAX &&
		i2c_data_size <= I2C_DATA_SIZE_MAX)
	{
		i2c_slave_addr = slave_addr;

		memcpy(i2c_param, param, param_size);
		i2c_param_size = param_size;

		i2c_data_size = data_size;

		i2c_mode = I2C_MODE_READ;

		i2c_begin();

		PT_WAIT_FOR(i2c_complete());

		if(!i2c_error)
		{
			memcpy(data, i2c_data, data_size);

			*status = I2C_STATUS_OK;
		}
		else
		{
			*status = I2C_STATUS_HW_ERROR;
		}
	}
	else
	{
		*status = I2C_STATUS_VALUE_ERROR;
	}

	pt_mutex_release(&mutex);

	PT_EXIT();
}

void i2c_init()
{
	static pt_t pt;

	LL_APB2_GRP1_EnableClock(SDA_PERIPH);
	LL_APB2_GRP1_EnableClock(SCL_PERIPH);

	GPIO_MODE(SDA_GPIO, SDA_PIN, LL_GPIO_MODE_OUTPUT);
	GPIO_OTYPE(SDA_GPIO, SDA_PIN, LL_GPIO_OUTPUT_OPENDRAIN);
	GPIO_SET(SDA_GPIO, SDA_PIN);
	GPIO_SPEED(SDA_GPIO, SDA_PIN, LL_GPIO_SPEED_FREQ_LOW);

	GPIO_MODE(SCL_GPIO, SCL_PIN, LL_GPIO_MODE_OUTPUT);
	GPIO_OTYPE(SCL_GPIO, SCL_PIN, LL_GPIO_OUTPUT_OPENDRAIN);
	GPIO_SET(SCL_GPIO, SCL_PIN);
	GPIO_SPEED(SCL_GPIO, SCL_PIN, LL_GPIO_SPEED_FREQ_LOW);

	pt_create(&pt, i2c_proc, 1);
}
