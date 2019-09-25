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
#include <drivers/button.h>
#include <mgmt/pwr_mgr.h>
#include <mgmt/example_mgr.h>
#include "sys_mgr.h"

static bool shutdown;
static bool charging_mode;

static
void sys_mgr_sleep()
{
	//
	// PWR clock enable
	//
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	//
	// Configure sleep mode
	//
	LL_PWR_SetPowerMode(LL_PWR_MODE_STANDBY);
	LL_PWR_ClearFlag_WU();
	LL_LPM_EnableDeepSleep();
//	LL_DBGMCU_EnableDBGStandbyMode();

	//
	// Good night, MCU...
	//
	__DSB();
	__WFE();
}

static
pt_status_t sys_mgr_proc()
{
	static tmr_t timer;
	button_ev_t ev;

	PT_ENTER();

	//
	// Early system initializing
	//
	timer_init();
	pwr_mgr_init();
	button_init();
	time_update(timer);

	//
	// Check power up condition
	//
	for(;;)
	{
		if(0/*is_charger_connected()*/)
		{
			charging_mode = 1;

			break;
		}

		if(button_event_get(&ev))
		{
			if(ev.btn_id == BUTTON_MIDDLE &&
			   ev.ev_code == BUTTON_EVENT_HOLD)
			{
				break;
			}
		}

		if(time_elapsed(timer) >= MS(500))
		{
			if(0/*batt_mgr_percent_get() == 0*/)
			{
				goto shutdown;
			}
		}

		if(time_elapsed(timer) >= MS(2000))
		{
			goto shutdown;
		}

		PT_YIELD();
	}

	//
	// Lock buttons until released
	//
	buttons_lock();

	//
	// Init remaining part of system
	//
	example_mgr_init();

	time_update(timer);

	//
	// Run until shutdown
	//
	while(!shutdown)
	{
		if(charging_mode)
		{
			if(1/*!is_charger_connected()*/)
			{
				if(time_elapsed(timer) >= MS(2000))
				{
					//
					// Device removed from charger
					//
					break;
				}
			}
			else
			{
				time_update(timer);
			}
		}
		else
		{
			if(/*!batt_mgr_percent_get()*/0 && /*!is_charger_connected()*/1)
			{
				if(time_elapsed(timer) >= MS(2000))
				{
					//
					// Battery depleted
					//
					break;
				}
			}
			else
			{
				time_update(timer);
			}
		}

		PT_YIELD();
	}

shutdown:
	//
	// Perform system shutdown
	//
//	batt_mgr_shutdown();

	//
	// Wait time for shutdown
	//
	time_sleep(timer, MS(200));

	//
	// Powerdown...
	//
	pwr_mgr_shutdown();
	sys_mgr_sleep();

	PT_EXIT();
}

void sys_mgr_shutdown()
{
	shutdown = 1;
}

void sys_mgr_charging_mode_disable()
{
	charging_mode = 0;
}

bool sys_mgr_is_in_charging_mode()
{
	return charging_mode;
}

void sys_mgr_init()
{
	static pt_t pt;

	pt_create(&pt, sys_mgr_proc, 0);
}
