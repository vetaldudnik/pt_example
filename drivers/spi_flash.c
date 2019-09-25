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
#include "spi_flash.h"

#define FLASH_SPI_PERIPH            LL_APB1_GRP1_PERIPH_SPI2
#define FLASH_SPIx                  SPI2

#define FLASH_CS_GPIO_PERIPH        LL_APB2_GRP1_PERIPH_GPIOB
#define FLASH_CS_GPIOx              GPIOB
#define FLASH_CS_PIN                LL_GPIO_PIN_12

#define FLASH_SCK_GPIO_PERIPH       LL_APB2_GRP1_PERIPH_GPIOB
#define FLASH_SCK_GPIOx             GPIOB
#define FLASH_SCK_PIN               LL_GPIO_PIN_13

#define FLASH_MOSI_GPIO_PERIPH      LL_APB2_GRP1_PERIPH_GPIOB
#define FLASH_MOSI_GPIOx            GPIOB
#define FLASH_MOSI_PIN              LL_GPIO_PIN_15

#define FLASH_MISO_GPIO_PERIPH      LL_APB2_GRP1_PERIPH_GPIOB
#define FLASH_MISO_GPIOx            GPIOB
#define FLASH_MISO_PIN              LL_GPIO_PIN_14

#define FLASH_RX_DMA_PERIPH         LL_AHB1_GRP1_PERIPH_DMA1
#define FLASH_RX_DMAx               DMA1
#define FLASH_RX_DMA_CHANNEL        LL_DMA_CHANNEL_4
#define FLASH_RX_DMA_IRQn           DMA1_Channel4_IRQn
#define FLASH_RX_DMA_IRQ            DMA1_Channel4_IRQHandler

#define FLASH_TX_DMA_PERIPH         LL_AHB1_GRP1_PERIPH_DMA1
#define FLASH_TX_DMAx               DMA1
#define FLASH_TX_DMA_CHANNEL        LL_DMA_CHANNEL_5

#define FLASH_CS_LOW()              LL_GPIO_ResetOutputPin(FLASH_CS_GPIOx, FLASH_CS_PIN);
#define FLASH_CS_HIGH()             LL_GPIO_SetOutputPin(FLASH_CS_GPIOx, FLASH_CS_PIN);

#define BLOCK_SIZE                  (  256 )

enum
{
	CMD_STATUS_WRITE  = 0x01,
	CMD_DATA_WRITE    = 0x02,
	CMD_DATA_READ     = 0x03,
	CMD_WRITE_DISABLE = 0x04,
	CMD_STATUS_READ   = 0x05,
	CMD_WRITE_ENABLE  = 0x06,
	CMD_JEDEC_ID_READ = 0x9F,
	CMD_SECT_ERASE    = 0x20,
	CMD_WHOLE_ERASE   = 0xC7,
};

static u08 flash_data[260];
static u32 rx_size;
static bool dma_spi_cmd_complete;
static pt_mutex_t mutex;

static
void dma_spi_perform_tx(void *data, u32 size)
{
	static u08 dummy;

	//
	// Configure RX DMA stream
	//
	LL_DMA_DisableChannel(FLASH_RX_DMAx, FLASH_RX_DMA_CHANNEL);
	LL_DMA_SetMemoryAddress(FLASH_RX_DMAx, FLASH_RX_DMA_CHANNEL, (u32)&dummy);
	LL_DMA_SetMemoryIncMode(FLASH_RX_DMAx, FLASH_RX_DMA_CHANNEL, LL_DMA_MEMORY_NOINCREMENT);
	LL_DMA_SetDataLength(FLASH_RX_DMAx, FLASH_RX_DMA_CHANNEL, size);
	LL_DMA_EnableChannel(FLASH_RX_DMAx, FLASH_RX_DMA_CHANNEL);

	//
	// Configure TX DMA stream
	//
	LL_DMA_DisableChannel(FLASH_TX_DMAx, FLASH_TX_DMA_CHANNEL);
	LL_DMA_SetMemoryAddress(FLASH_TX_DMAx, FLASH_TX_DMA_CHANNEL, (u32)data);
	LL_DMA_SetMemoryIncMode(FLASH_TX_DMAx, FLASH_TX_DMA_CHANNEL, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetDataLength(FLASH_TX_DMAx, FLASH_TX_DMA_CHANNEL, size);
	LL_DMA_EnableChannel(FLASH_TX_DMAx, FLASH_TX_DMA_CHANNEL);
}

static
void dma_spi_perform_rx(void *data, u32 size)
{
	static u08 dummy;

	//
	// Configure RX DMA stream
	//
	LL_DMA_DisableChannel(FLASH_RX_DMAx, FLASH_RX_DMA_CHANNEL);
	LL_DMA_SetMemoryAddress(FLASH_RX_DMAx, FLASH_RX_DMA_CHANNEL, (u32)data);
	LL_DMA_SetMemoryIncMode(FLASH_RX_DMAx, FLASH_RX_DMA_CHANNEL, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetDataLength(FLASH_RX_DMAx, FLASH_RX_DMA_CHANNEL, size);
	LL_DMA_EnableChannel(FLASH_RX_DMAx, FLASH_RX_DMA_CHANNEL);

	//
	// Configure TX DMA stream
	//
	LL_DMA_DisableChannel(FLASH_TX_DMAx, FLASH_TX_DMA_CHANNEL);
	LL_DMA_SetMemoryAddress(FLASH_TX_DMAx, FLASH_TX_DMA_CHANNEL, (u32)&dummy);
	LL_DMA_SetMemoryIncMode(FLASH_TX_DMAx, FLASH_TX_DMA_CHANNEL, LL_DMA_MEMORY_NOINCREMENT);
	LL_DMA_SetDataLength(FLASH_TX_DMAx, FLASH_TX_DMA_CHANNEL, size);
	LL_DMA_EnableChannel(FLASH_TX_DMAx, FLASH_TX_DMA_CHANNEL);
}

static
bool is_dma_spi_cmd_complete()
{
	bool complete = 0;

	if(dma_spi_cmd_complete)
	{
		dma_spi_cmd_complete = 0;

		complete = 1;
	}

	return complete;
}

void FLASH_RX_DMA_IRQ()
{
	static u08 stage;

	LL_DMA_ClearFlag_TC4(FLASH_RX_DMAx);

	switch(stage)
	{
	case 0:
		if(rx_size != 0)
		{
			dma_spi_perform_rx(flash_data, rx_size);

			stage++;

			break;
		}

		/* no break */

	case 1:
		FLASH_CS_HIGH();

		dma_spi_cmd_complete = 1;

		stage = 0;

		break;
	}
}

pt_status_t spi_flash_id_read(u08 *buf, u32 size)
{
	u32 tx_size;

	if(!pt_mutex_acquire(&mutex))
	{
		return PT_STATUS_YIELD;
	}

	PT_ENTER();

	tx_size = 0;

	flash_data[tx_size++] = CMD_JEDEC_ID_READ;

	rx_size = size;

	FLASH_CS_LOW();

	dma_spi_perform_tx(flash_data, tx_size);

	PT_WAIT_FOR(is_dma_spi_cmd_complete());

	memcpy(buf, flash_data, rx_size);

	pt_mutex_release(&mutex);

	PT_EXIT();
}

pt_status_t spi_flash_data_read(u32 addr, void *buf, u32 size)
{
	static u32 remain;
	static u32 data_addr;
	static void *data_buf;
	static u32 data_size;
	u32 block_remain;
	u32 tx_size;

	if(!pt_mutex_acquire(&mutex))
	{
		return PT_STATUS_YIELD;
	}

	PT_ENTER();

	data_addr = addr;
	data_buf  = buf;
	remain    = size;

	while(remain)
	{
		tx_size = 0;

		flash_data[tx_size++] = CMD_DATA_READ;
		flash_data[tx_size++] = (data_addr >> 16) & 0xFF;
		flash_data[tx_size++] = (data_addr >>  8) & 0xFF;
		flash_data[tx_size++] = (data_addr >>  0) & 0xFF;

		block_remain = BLOCK_SIZE - (data_addr & (BLOCK_SIZE - 1));

		data_size = (remain >= block_remain) ? block_remain : remain;

		rx_size = data_size;

		FLASH_CS_LOW();

		dma_spi_perform_tx(flash_data, tx_size);

		PT_WAIT_FOR(is_dma_spi_cmd_complete());

		memcpy(data_buf, flash_data, data_size);

		data_buf  += data_size;
		data_addr += data_size;
		remain    -= data_size;
	}

	pt_mutex_release(&mutex);

	PT_EXIT();
}

pt_status_t spi_flash_data_write(u32 addr, const void *buf, u32 size)
{
	static u32 remain;
	static u32 data_addr;
	static const void *data_buf;
	static u32 data_size;
	u32 block_remain;
	u32 tx_size;

	if(!pt_mutex_acquire(&mutex))
	{
		return PT_STATUS_YIELD;
	}

	PT_ENTER();

	//
	// Send write enable command
	//
	{
		tx_size = 0;

		flash_data[tx_size++] = CMD_WRITE_ENABLE;

		rx_size = 0;

		FLASH_CS_LOW();

		dma_spi_perform_tx(flash_data, tx_size);

		PT_WAIT_FOR(is_dma_spi_cmd_complete());
	}

	data_addr = addr;
	data_buf  = buf;
	remain    = size;

	while(remain)
	{
		tx_size = 0;

		flash_data[tx_size++] = CMD_DATA_WRITE;
		flash_data[tx_size++] = (data_addr >> 16) & 0xFF;
		flash_data[tx_size++] = (data_addr >>  8) & 0xFF;
		flash_data[tx_size++] = (data_addr >>  0) & 0xFF;

		block_remain = BLOCK_SIZE - (data_addr & (BLOCK_SIZE - 1));

		data_size = (remain >= block_remain) ? block_remain : remain;

		memcpy(&flash_data[tx_size], data_buf, data_size);

		tx_size += data_size;

		rx_size = 0;

		FLASH_CS_LOW();

		dma_spi_perform_tx(flash_data, tx_size);

		PT_WAIT_FOR(is_dma_spi_cmd_complete());

		//
		// Check for write complete
		//
		do
		{
			tx_size = 0;

			flash_data[tx_size++] = CMD_STATUS_READ;

			rx_size = 1;

			FLASH_CS_LOW();

			dma_spi_perform_tx(flash_data, tx_size);

			PT_WAIT_FOR(is_dma_spi_cmd_complete());
		}
		while(flash_data[0] & (1 << 0));

		data_buf  += data_size;
		data_addr += data_size;
		remain    -= data_size;
	}

	pt_mutex_release(&mutex);

	PT_EXIT();
}

pt_status_t spi_flash_sector_erase(u32 addr)
{
	u32 tx_size;

	if(!pt_mutex_acquire(&mutex))
	{
		return PT_STATUS_YIELD;
	}

	PT_ENTER();

	//
	// Send write enable command
	//
	{
		tx_size = 0;

		flash_data[tx_size++] = CMD_WRITE_ENABLE;

		rx_size = 0;

		FLASH_CS_LOW();

		dma_spi_perform_tx(flash_data, tx_size);

		PT_WAIT_FOR(is_dma_spi_cmd_complete());
	}

	//
	// Send sector erase command
	//
	{
		tx_size = 0;

		flash_data[tx_size++] = CMD_SECT_ERASE;
		flash_data[tx_size++] = (addr >> 16) & 0xFF;
		flash_data[tx_size++] = (addr >>  8) & 0xFF;
		flash_data[tx_size++] = (addr >>  0) & 0xFF;

		rx_size = 0;

		FLASH_CS_LOW();

		dma_spi_perform_tx(flash_data, tx_size);

		PT_WAIT_FOR(is_dma_spi_cmd_complete());
	}

	//
	// Check for write complete
	//
	do
	{
		tx_size = 0;

		flash_data[tx_size++] = CMD_STATUS_READ;

		rx_size = 1;

		FLASH_CS_LOW();

		dma_spi_perform_tx(flash_data, tx_size);

		PT_WAIT_FOR(is_dma_spi_cmd_complete());
	}
	while(flash_data[0] & (1 << 0));

	pt_mutex_release(&mutex);

	PT_EXIT();
}

pt_status_t spi_flash_whole_erase()
{
	u32 tx_size;

	if(!pt_mutex_acquire(&mutex))
	{
		return PT_STATUS_YIELD;
	}

	PT_ENTER();

	//
	// Send write enable command
	//
	{
		tx_size = 0;

		flash_data[tx_size++] = CMD_WRITE_ENABLE;

		rx_size = 0;

		FLASH_CS_LOW();

		dma_spi_perform_tx(flash_data, tx_size);

		PT_WAIT_FOR(is_dma_spi_cmd_complete());
	}

	//
	// Send sector erase command
	//
	{
		tx_size = 0;

		flash_data[tx_size++] = CMD_WHOLE_ERASE;

		rx_size = 0;

		FLASH_CS_LOW();

		dma_spi_perform_tx(flash_data, tx_size);

		PT_WAIT_FOR(is_dma_spi_cmd_complete());
	}

	//
	// Check for write complete
	//
	do
	{
		tx_size = 0;

		flash_data[tx_size++] = CMD_STATUS_READ;

		rx_size = 1;

		FLASH_CS_LOW();

		dma_spi_perform_tx(flash_data, tx_size);

		PT_WAIT_FOR(is_dma_spi_cmd_complete());
	}
	while(flash_data[0] & (1 << 0));

	pt_mutex_release(&mutex);

	PT_EXIT();
}

void spi_flash_init()
{
	LL_GPIO_InitTypeDef GPIO_InitStruct;
	LL_DMA_InitTypeDef  DMA_InitStruct;
	LL_SPI_InitTypeDef  SPI_InitStruct;

	//
	// GPIO clocks enable
	//
	LL_APB2_GRP1_EnableClock(
			FLASH_CS_GPIO_PERIPH   |
			FLASH_SCK_GPIO_PERIPH  |
			FLASH_MOSI_GPIO_PERIPH |
			FLASH_MISO_GPIO_PERIPH);

	//
	// GPIO common init
	//
	LL_GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;

	//
	// SCK GPIO init
	//
	GPIO_InitStruct.Pin = FLASH_SCK_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	LL_GPIO_Init(FLASH_SCK_GPIOx, &GPIO_InitStruct);

	//
	// MOSI GPIO init
	//
	GPIO_InitStruct.Pin = FLASH_MOSI_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	LL_GPIO_Init(FLASH_MOSI_GPIOx, &GPIO_InitStruct);

	//
	// MISO GPIO init
	//
	GPIO_InitStruct.Pin = FLASH_MISO_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	LL_GPIO_Init(FLASH_MISO_GPIOx, &GPIO_InitStruct);

	//
	// CS GPIO init
	//
	GPIO_InitStruct.Pin = FLASH_CS_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	LL_GPIO_Init(FLASH_CS_GPIOx, &GPIO_InitStruct);
	LL_GPIO_SetOutputPin(FLASH_CS_GPIOx, FLASH_CS_PIN);

	//
	// DMA clocks enable
	//
	LL_AHB1_GRP1_EnableClock(FLASH_RX_DMA_PERIPH | FLASH_TX_DMA_PERIPH);

	//
	// DMA common init
	//
	LL_DMA_StructInit(&DMA_InitStruct);
	DMA_InitStruct.PeriphOrM2MSrcAddress = LL_SPI_DMA_GetRegAddr(FLASH_SPIx);
	DMA_InitStruct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
	DMA_InitStruct.MemoryOrM2MDstAddress = 0;
	DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_PERIPH_NOINCREMENT;
	DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_MDATAALIGN_BYTE;
	DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
	DMA_InitStruct.Mode = LL_DMA_MODE_NORMAL;
	DMA_InitStruct.NbData = 0;
	DMA_InitStruct.Priority = LL_DMA_PRIORITY_MEDIUM;

	//
	// SPI RX DMA init
	//
	DMA_InitStruct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
	LL_DMA_Init(FLASH_RX_DMAx, FLASH_RX_DMA_CHANNEL, &DMA_InitStruct);

	//
	// SPI TX DMA init
	//
	DMA_InitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
	LL_DMA_Init(FLASH_TX_DMAx, FLASH_TX_DMA_CHANNEL, &DMA_InitStruct);

	//
	// DMA transfer complete interrupt enable
	//
	LL_DMA_EnableIT_TC(FLASH_RX_DMAx, FLASH_RX_DMA_CHANNEL);

	//
	// NVIC config
	//
	NVIC_SetPriority(FLASH_RX_DMA_IRQn, 3);
	NVIC_EnableIRQ(FLASH_RX_DMA_IRQn);

	//
	// SPI init
	//
	LL_APB1_GRP1_EnableClock(FLASH_SPI_PERIPH);
	LL_SPI_StructInit(&SPI_InitStruct);
	SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
	SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	LL_SPI_Init(FLASH_SPIx, &SPI_InitStruct);

	//
	// SPI DMA request enable
	//
	LL_SPI_EnableDMAReq_TX(FLASH_SPIx);
	LL_SPI_EnableDMAReq_RX(FLASH_SPIx);

	//
	// Enable SPI
	//
	LL_SPI_Enable(FLASH_SPIx);
}
