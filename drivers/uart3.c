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
#include "uart.h"

#define USARTx                      USART3
#define USARTx_PERIPH               LL_APB1_GRP1_PERIPH_USART3
#define USARTx_IRQn                 USART3_IRQn
#define USARTx_IRQHandler           USART3_IRQHandler

#define USARTx_GPIO                 GPIOB
#define USARTx_GPIO_PERIPH          LL_APB2_GRP1_PERIPH_GPIOB

#define USARTx_TX_PIN               LL_GPIO_PIN_10
#define USARTx_RX_PIN               LL_GPIO_PIN_11

static char rx_queue_buf[256];
static char tx_queue_buf[256];

static queue_t rx_queue;
static queue_t tx_queue;

static volatile tmr_t rx_timer;

static bool tx_complete;

static
bool uart3_is_tx_complete()
{
	return tx_complete;
}

void USARTx_IRQHandler()
{
	unsigned char data;

	//
	// Receiver not empty
	//
	if(LL_USART_IsEnabledIT_RXNE(USARTx) &&
		LL_USART_IsActiveFlag_RXNE(USARTx))
	{
		data = LL_USART_ReceiveData8(USARTx);

		queue_send(&rx_queue, &data);

		time_update(rx_timer);
	}

	//
	// Transmitter empty
	//
	if(LL_USART_IsEnabledIT_TXE(USARTx) &&
		LL_USART_IsActiveFlag_TXE(USARTx))
	{
		if(queue_recv(&tx_queue, &data))
		{
			LL_USART_TransmitData8(USARTx, data);
		}
		else
		{
			LL_USART_DisableIT_TXE(USARTx);
			LL_USART_EnableIT_TC(USARTx);
		}
	}

	//
	// Transmission complete
	//
	if(LL_USART_IsEnabledIT_TC(USARTx) &&
		LL_USART_IsActiveFlag_TC(USARTx))
	{
		tx_complete = 1;

		LL_USART_DisableIT_TC(USARTx);
	}
}

pt_status_t uart3_recv(void *data, int size, int *recvd, tmr_t timeout)
{
	static int remain;
	static int count;
	tmr_t elapsed;

	PT_ENTER();

	remain = size;
	count = 0;

	time_update(rx_timer);

	while(remain)
	{
		while(queue_recv(&rx_queue, data + count))
		{
			count++;

			if(--remain == 0)
			{
				break;
			}
		}

		if(timeout != -1)
		{
			LL_USART_DisableIT_RXNE(USARTx);
			__DSB();
			__ISB();

			elapsed = time_elapsed(rx_timer);

			LL_USART_EnableIT_RXNE(USARTx);

			if(elapsed >= timeout)
			{
				break;
			}
		}

		PT_YIELD();
	}

	*recvd = count;

	PT_EXIT();
}

pt_status_t uart3_send(const void *data, int size)
{
	static int remain;
	static int count;

	PT_ENTER();

	remain = size;
	count = 0;

	while(remain)
	{
		while(queue_send(&tx_queue, data + count))
		{
			tx_complete = 0;

			count++;

			LL_USART_EnableIT_TXE(USARTx);

			if(--remain == 0)
			{
				break;
			}
		}

		PT_YIELD();
	}

	PT_WAIT_FOR(uart3_is_tx_complete());

	PT_EXIT();
}

void uart3_configure(u32 baudrate, uart_parity_t parity)
{
	LL_USART_InitTypeDef USART_InitStruct;

	LL_USART_Disable(USARTx);

	LL_USART_StructInit(&USART_InitStruct);
	USART_InitStruct.BaudRate = baudrate;
	USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;

	switch(parity)
	{
	case UART_PARITY_NONE:
		USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
		USART_InitStruct.Parity = LL_USART_PARITY_NONE;
		break;

	case UART_PARITY_EVEN:
		USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_9B;
		USART_InitStruct.Parity = LL_USART_PARITY_EVEN;
		break;

	case UART_PARITY_ODD:
		USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_9B;
		USART_InitStruct.Parity = LL_USART_PARITY_ODD;
		break;
	}

	LL_USART_Init(USARTx, &USART_InitStruct);

	LL_USART_Enable(USARTx);
}

void uart3_init()
{
	LL_USART_InitTypeDef USART_InitStruct;
	LL_GPIO_InitTypeDef GPIO_InitStruct;

	/* Init queues */
	queue_init(&rx_queue, sizeof(rx_queue_buf[0]), sizeof(rx_queue_buf) / sizeof(rx_queue_buf[0]), rx_queue_buf);
	queue_init(&tx_queue, sizeof(tx_queue_buf[0]), sizeof(tx_queue_buf) / sizeof(tx_queue_buf[0]), tx_queue_buf);

	/* UART clock enable */
	LL_APB1_GRP1_EnableClock(USARTx_PERIPH);

	/* GPIOs clock enable */
	LL_APB2_GRP1_EnableClock(USARTx_GPIO_PERIPH);

	/* GPIO TX init */
	LL_GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.Pin = USARTx_TX_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO_Init(USARTx_GPIO, &GPIO_InitStruct);

	/* GPIO RX init */
	LL_GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.Pin = USARTx_RX_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Init(USARTx_GPIO, &GPIO_InitStruct);

	/* UART init (defaults) */
	LL_USART_StructInit(&USART_InitStruct);
	LL_USART_Init(USARTx, &USART_InitStruct);

	/* Enable UART global interrupt */
	NVIC_SetPriority(USARTx_IRQn, 0);
	NVIC_EnableIRQ(USARTx_IRQn);

	/* Enable RXNE interrupt */
	LL_USART_EnableIT_RXNE(USARTx);

	/* Enable UART */
	LL_USART_Enable(USARTx);
}
