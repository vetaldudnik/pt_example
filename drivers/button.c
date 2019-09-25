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
#include <drivers/gpio.h>
#include "button.h"

#define UP_BTN_GPIO_PERIPH          LL_APB2_GRP1_PERIPH_GPIOA
#define UP_BTN_GPIO_PORT            GPIOA
#define UP_BTN_GPIO_PIN             LL_GPIO_PIN_11

#define START_BTN_GPIO_PERIPH       LL_APB2_GRP1_PERIPH_GPIOA
#define START_BTN_GPIO_PORT         GPIOA
#define START_BTN_GPIO_PIN          LL_GPIO_PIN_12

#define DOWN_BTN_GPIO_PERIPH        LL_APB2_GRP1_PERIPH_GPIOA
#define DOWN_BTN_GPIO_PORT          GPIOA
#define DOWN_BTN_GPIO_PIN           LL_GPIO_PIN_10

#define BUTTON_HOLD_DELAY_MS        ( 1000 )
#define BUTTON_AUTOREPEAT_DELAY_MS  (  100 )

typedef enum
{
	PIN_STATE_UNKNOWN = -1,
	PIN_STATE_LOW,
	PIN_STATE_HIGH,

} pin_state_t;

typedef enum
{
	BUTTON_STAGE_RELEASED,
	BUTTON_STAGE_PRESSED,
	BUTTON_STAGE_HOLD,
	BUTTON_STAGE_LOCKED,

} button_stage_t;

typedef struct
{
	uint32_t gpio_periph;
	GPIO_TypeDef* gpio_port;
	uint32_t gpio_pin;

} button_hwcfg_t;

typedef struct
{
	pin_state_t pin_state;
	button_stage_t stage;
	uint8_t  debounce_cnt;
	uint16_t timer;

} button_state_t;

static const button_hwcfg_t button_hwcfg[] =
{
	{ UP_BTN_GPIO_PERIPH,    UP_BTN_GPIO_PORT,    UP_BTN_GPIO_PIN    },
	{ START_BTN_GPIO_PERIPH, START_BTN_GPIO_PORT, START_BTN_GPIO_PIN },
	{ DOWN_BTN_GPIO_PERIPH,  DOWN_BTN_GPIO_PORT,  DOWN_BTN_GPIO_PIN  },
};

#define BUTTONS_COUNT               (sizeof(button_hwcfg) / \
                                     sizeof(button_hwcfg[0]))

static button_state_t button_state[BUTTONS_COUNT];
static queue_t        button_event_queue;
static button_ev_t    button_event_queue_buf[8];
static bool           buttons_flipped;

static
button_id_t button_id_remap(button_id_t id)
{
	const button_id_t remap_tbl[] =
	{
		BUTTON_DOWN,
		BUTTON_MIDDLE,
		BUTTON_UP
	};

	if(buttons_flipped)
	{
		return remap_tbl[id];
	}

	return id;
}

static
void button_hw_state_proc()
{
	static unsigned char prev_pin_state[BUTTONS_COUNT];

	for(button_id_t i = 0; i < BUTTONS_COUNT; i++)
	{
		unsigned char curr_pin_state = GPIO_STATE(button_hwcfg[i].gpio_port, button_hwcfg[i].gpio_pin);

		if(prev_pin_state[i] != curr_pin_state)
		{
			prev_pin_state[i] = curr_pin_state;

			button_state[i].debounce_cnt = 0;
		}
		else
		{
			//
			// Debounce check
			//
			if(++button_state[i].debounce_cnt >= 10)
			{
				button_state[i].pin_state = curr_pin_state ? PIN_STATE_HIGH : PIN_STATE_LOW;

				button_state[i].debounce_cnt = 0;
			}
		}
	}
}

static
void button_sw_state_proc()
{
	static uint16_t ms_tick;
	static uint8_t btn_id;
	uint8_t pressed;
	button_ev_t ev;

	//
	// Tick increment
	//
	ms_tick++;

	if(button_state[btn_id].pin_state != PIN_STATE_UNKNOWN)
	{
		//
		// Get current button pin state
		//
		if(btn_id == BUTTON_MIDDLE)
		{
			pressed = button_state[btn_id].pin_state ? 1 : 0;
		}
		else
		{
			pressed = button_state[btn_id].pin_state ? 0 : 1;
		}

		switch(button_state[btn_id].stage)
		{
		case BUTTON_STAGE_RELEASED:
			if(pressed)
			{
				//
				// Update button timer
				//
				button_state[btn_id].timer = ms_tick;

				//
				// Change button stage
				//
				button_state[btn_id].stage = BUTTON_STAGE_PRESSED;

				//
				// Make event
				//
				ev.ev_code = BUTTON_EVENT_PRESS;
				ev.btn_id  = button_id_remap(btn_id);

				//
				// Send event
				//
				queue_send(&button_event_queue, &ev);
			}

			break;

		case BUTTON_STAGE_PRESSED:
			if(pressed)
			{
				//
				// Wait for button hold delay
				//
				if((ms_tick - button_state[btn_id].timer) >= BUTTON_HOLD_DELAY_MS)
				{
					//
					// Update button timer
					//
					button_state[btn_id].timer = ms_tick;

					//
					// Change button stage
					//
					button_state[btn_id].stage = BUTTON_STAGE_HOLD;

					//
					// Make event
					//
					ev.ev_code = BUTTON_EVENT_HOLD;
					ev.btn_id  = button_id_remap(btn_id);

					//
					// Send event
					//
					queue_send(&button_event_queue, &ev);
				}
			}
			else
			{
				//
				// Change button stage
				//
				button_state[btn_id].stage = BUTTON_STAGE_RELEASED;

				//
				// Make event
				//
				ev.ev_code = BUTTON_EVENT_RELEASE;
				ev.btn_id  = button_id_remap(btn_id);

				//
				// Send event
				//
				queue_send(&button_event_queue, &ev);
			}

			break;

		case BUTTON_STAGE_HOLD:
			if(pressed)
			{
				//
				// Wait for button autorepeat delay
				//
				if((ms_tick - button_state[btn_id].timer) >= BUTTON_AUTOREPEAT_DELAY_MS )
				{
					//
					// Update button timer
					//
					button_state[btn_id].timer = ms_tick;

					//
					// Make event
					//
					ev.ev_code = BUTTON_EVENT_AUTOREPEAT;
					ev.btn_id  = button_id_remap(btn_id);

					//
					// Send event
					//
					queue_send(&button_event_queue, &ev);
				}
			}
			else
			{
				//
				// Change button stage
				//
				button_state[btn_id].stage = BUTTON_STAGE_RELEASED;

//				//
//				// Make event
//				//
//				ev.ev_code = EVENT_BUTTON_RELEASE;
//				ev.btn_id  = button_id_remap(btn_id);
//
//				//
//				// Send event
//				//
//				queue_send(&button_event_queue, &ev);
			}

			break;

		case BUTTON_STAGE_LOCKED:
			if(!pressed)
			{
				//
				// Change button stage
				//
				button_state[btn_id].stage = BUTTON_STAGE_RELEASED;
			}

			break;
		}
	}

	//
	// Next button
	//
	if(++btn_id >= BUTTONS_COUNT)
	{
		btn_id = 0;
	}
}

static
pt_status_t button_proc()
{
	static tmr_t timer;

	PT_ENTER();

	for(;;)
	{
		button_hw_state_proc();
		button_sw_state_proc();

		time_sleep(timer, MS(1));
	}

	PT_EXIT();
}

void buttons_flip_enable(bool enable)
{
	buttons_flipped = enable;
}

void buttons_lock()
{
	button_ev_t ev;

	for(int i = 0; i < BUTTONS_COUNT; i++)
	{
		button_state[i].stage = BUTTON_STAGE_LOCKED;
	}

	while(queue_recv(&button_event_queue, &ev));
}

int button_event_get(button_ev_t *ev)
{
	if(queue_recv(&button_event_queue, ev))
	{
		return 1;
	}

	return 0;
}

void button_init()
{
	static pt_t pt;

	LL_GPIO_InitTypeDef GPIO_InitStruct;

	//
	// GPIO common init
	//
	LL_GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;

	for(button_id_t i = 0; i < BUTTONS_COUNT; i++)
	{
		//
		// Clock enable
		//
		LL_APB2_GRP1_EnableClock(button_hwcfg[i].gpio_periph);

		//
		// Get led pin number
		//
		GPIO_InitStruct.Pin = button_hwcfg[i].gpio_pin;

		//
		// GPIO init
		//
		LL_GPIO_Init(button_hwcfg[i].gpio_port, &GPIO_InitStruct);

		//
		// Set initial pin state
		//
		button_state[i].pin_state = PIN_STATE_UNKNOWN;
	}

    //
    // Init keypad event queue
    //
    int item_size  = sizeof(button_event_queue_buf[0]);
    int item_count = sizeof(button_event_queue_buf) / item_size;

    queue_init(&button_event_queue, item_size, item_count, button_event_queue_buf);

    pt_create(&pt, button_proc, 1);
}
