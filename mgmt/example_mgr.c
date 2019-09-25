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
#include <drivers/spi_flash.h>
#include <drivers/uart.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>

#include "example_mgr.h"

static bool shutdown;
static u08 uart_data[256];

static u08 i2c_data[] =
{
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9
};

static
pt_status_t example_mgr_proc()
{
	u08 i2c_slave_addr = 0x15;
	i2c_status_t i2c_status;
	u08 i2c_reg = 0x55;
	int recvd;

	//
	// All variable declarations must be placed before PT_ENTER()
	//
	PT_ENTER();

	//
	// Perform full flash erase
	//
	PT_WAIT_FOR(spi_flash_whole_erase());

	//
	// Run until shutdown
	//
	while(!shutdown)
	{
		PT_WAIT_FOR(uart3_recv(uart_data, sizeof(uart_data), &recvd, MS(1000)));

		if(recvd == sizeof(uart_data))
		{
			PT_WAIT_FOR(spi_flash_data_write(0x1000, uart_data, sizeof(uart_data)));

			PT_WAIT_FOR(i2c_write(i2c_slave_addr, &i2c_reg, sizeof(i2c_reg), i2c_data, sizeof(i2c_data), &i2c_status));

			if(i2c_status == I2C_STATUS_OK)
			{
				PT_WAIT_FOR(i2c_read(i2c_slave_addr, &i2c_reg, sizeof(i2c_reg), i2c_data, sizeof(i2c_data), &i2c_status));

				if(i2c_status == I2C_STATUS_OK)
				{
					PT_WAIT_FOR(spi_flash_data_write(0x2000, i2c_data, sizeof(i2c_data)));

					goto complete;
				}
			}
		}

		//
		// PT_YIELD() must be present at the end of infinity loop if none PT_WAIT_FOR() present above
		//
//		PT_YIELD();
	}

complete:
	PT_EXIT();
}

void example_mgr_shutdown()
{
	shutdown = 1;
}

void example_mgr_init()
{
	static pt_t pt;

	spi_flash_init();
	uart3_init();
	i2c_init();

	pt_create(&pt, example_mgr_proc, 0);
}
