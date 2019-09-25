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
#include "queue.h"

//*****************************************************************************
//
// Enqueues the new item
//
//*****************************************************************************
int queue_send(queue_t *q, const void *data)
{
	//
	// Compute the next value for the write pointer
	//
	int next = (q->write_ptr + 1) & (q->item_count - 1);

	//
	// Is the message queue full?
	//
	if(next != q->read_ptr)
	{
		//
		// Write this message into the next location in the message queue
		//
		memcpy(q->buffer + q->write_ptr * q->item_size, data, q->item_size);

		//
		// Update the message queue write pointer
		//
		q->write_ptr = next;

		//
		// Success
		//
		return 1;
	}

	return 0;
}

//*****************************************************************************
//
// Dequeues the item
//
//*****************************************************************************
int queue_recv(queue_t *q, void *data)
{
	//
	// Is any message present in queue?
	//
	if(q->read_ptr != q->write_ptr)
	{
		//
		// Copy the contents of this message
		//
		memcpy(data, q->buffer + q->read_ptr * q->item_size, q->item_size);

		//
		// Remove this message from the queue
		//
		q->read_ptr = (q->read_ptr + 1) & (q->item_count - 1);

		//
		// Success
		//
		return 1;
	}

	return 0;
}

//*****************************************************************************
//
// Init the queue
//
// Note: item_count must be power of two!!!
//
//*****************************************************************************
void queue_init(queue_t *q, int item_size, int item_count, void *buffer)
{
	assert((item_count & (item_count - 1)) == 0);

	q->item_size  = item_size;
	q->item_count = item_count;
	q->read_ptr   = 0;
	q->write_ptr  = 0;
	q->buffer     = buffer;
}
