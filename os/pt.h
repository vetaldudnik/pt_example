#ifndef _PT_H_
#define _PT_H_

/*
 * Copyright (c) 2004-2005, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Authors:
 *	Adam Dunkels <adam@sics.se>
 *	Vitaliy Dudnik <masterv.play@gmail.com>
 *
 */

//*****************************************************************************
//
// Make label
//
//*****************************************************************************
#define CONCAT(s1, s2) s1##s2
#define LABEL(s) CONCAT(L, s)

//*****************************************************************************
//
// Protothread start
//
//*****************************************************************************
#define PT_ENTER()                  \
static void *_stage;                \
char _need_yield = 0;               \
(void) _need_yield;                 \
do                                  \
{                                   \
	if(_stage)                      \
	{                               \
		goto *_stage;               \
	}                               \
}                                   \
while(0)

//*****************************************************************************
//
// Protothread exit
//
//*****************************************************************************
#define PT_EXIT()                   \
do                                  \
{                                   \
	_stage = 0;                     \
	return PT_STATUS_DONE;          \
}                                   \
while(0)

//*****************************************************************************
//
// Protothread wait for condition
//
//*****************************************************************************
#define PT_WAIT_FOR(cond)           \
 do                                 \
 {                                  \
	_stage = &&LABEL(__LINE__);     \
LABEL(__LINE__):                    \
	if(!(cond))                     \
	{                               \
		return PT_STATUS_YIELD;     \
	}                               \
}                                   \
while(0)

//*****************************************************************************
//
// Protothread yield
//
//*****************************************************************************
#define PT_YIELD()                  \
 do                                 \
 {                                  \
	 _need_yield = 1;               \
	_stage = &&LABEL(__LINE__);     \
LABEL(__LINE__):                    \
	if(_need_yield)                 \
	{                               \
		return PT_STATUS_YIELD;     \
	}                               \
}                                   \
while(0)

//*****************************************************************************
//
// Protothread return status
//
//*****************************************************************************
typedef enum
{
	PT_STATUS_YIELD = 0,
	PT_STATUS_DONE  = 1

} pt_status_t;

//*****************************************************************************
//
// Protothread control structure
//
//*****************************************************************************
typedef struct _pt
{
	struct _pt *next;
	pt_status_t (*function)();

} pt_t;

//*****************************************************************************
//
// Protothread mutex
//
//*****************************************************************************
typedef struct
{
	pt_t *owner;

} pt_mutex_t;

void pt_enter_critical();
void pt_exit_critical();
bool pt_mutex_acquire(pt_mutex_t *mutex);
void pt_mutex_release(pt_mutex_t *mutex);
void pt_schedule(u08 prio);
void pt_create(pt_t *pt, pt_status_t (*func)(), u08 prio);

#endif /* _PT_H_ */
