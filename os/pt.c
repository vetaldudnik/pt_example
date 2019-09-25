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
#include "pt.h"

#define PT_PRIO_COUNT               ( 2 )

static pt_t *pt_list[PT_PRIO_COUNT];
static pt_t *pt_running[PT_PRIO_COUNT];

static
pt_t *pt_running_get()
{
	int prio = PT_PRIO_COUNT;

	while(--prio >= 0)
	{
		if(pt_running[prio] != 0)
		{
			return pt_running[prio];
		}
	}

	return 0;
}

static
void pt_list_item_push(pt_t **head, pt_t *item)
{
	assert(head != 0);
	assert(item != 0);

	if(*head == 0)
	{
		item->next = 0;

		*head = item;
	}
	else
	{
		for(pt_t *p = *head; p != item; p = p->next)
		{
			if(p->next == 0)
			{
				item->next = 0;

				p->next = item;

				break;
			}
		}
	}
}

static
void pt_list_item_remove(pt_t **head, pt_t *item)
{
	assert(head != 0);
	assert(item != 0);

	if(*head == item)
	{
		*head = item->next;
	}
	else
	{
		for(pt_t *p = *head; p->next; p = p->next)
		{
			if(p->next == item)
			{
				p->next = item->next;

				break;
			}
		}
	}
}

void pt_enter_critical()
{
	__disable_irq();

    __DSB();
	__ISB();
}

void pt_exit_critical()
{
	__enable_irq();
}

bool pt_mutex_acquire(pt_mutex_t *mutex)
{
	pt_t *pt = pt_running_get();
	pt_t *desired = pt;
	pt_t *expected = 0;

	assert(pt != 0);

	if(mutex->owner == pt)
	{
		return 1;
	}

	return __atomic_compare_exchange(&(mutex->owner), &expected, &desired, 0, 0, 0);
}

void pt_mutex_release(pt_mutex_t *mutex)
{
	pt_t *pt = pt_running_get();

	assert(pt != 0);

	if(mutex->owner == pt)
	{
		mutex->owner = 0;
	}
}

void pt_schedule(u08 prio)
{
	assert(prio < PT_PRIO_COUNT);

	for(pt_t *pt = pt_list[prio]; pt; pt = pt->next)
	{
		pt_running[prio] = pt;

		if(pt->function() == PT_STATUS_DONE)
		{
			pt_list_item_remove(&pt_list[prio], pt);
		}
	}

	pt_running[prio] = 0;
}

void pt_create(pt_t *pt, pt_status_t (*func)(), u08 prio)
{
	assert(pt != 0);
	assert(func != 0);
	assert(prio < PT_PRIO_COUNT);

	pt->function = func;

	pt_enter_critical();

	pt_list_item_push(&pt_list[prio], pt);

	pt_exit_critical();
}
