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

#ifndef DRIVERS_BUTTON_H_
#define DRIVERS_BUTTON_H_


typedef enum
{
	BUTTON_UP,
	BUTTON_MIDDLE,
	BUTTON_DOWN,

} button_id_t;


typedef enum
{
	BUTTON_EVENT_PRESS,
	BUTTON_EVENT_HOLD,
	BUTTON_EVENT_AUTOREPEAT,
	BUTTON_EVENT_RELEASE,

} button_ev_code_t;

typedef struct
{
	button_ev_code_t ev_code;
	button_id_t btn_id;

} button_ev_t;

void buttons_flip_enable(bool enable);
void buttons_lock();
int  button_event_get(button_ev_t *ev);
void button_init();


#endif /* DRIVERS_BUTTON_H_ */
