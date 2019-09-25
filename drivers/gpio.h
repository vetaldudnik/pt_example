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

#ifndef DRIVERS_GPIO_H_
#define DRIVERS_GPIO_H_

static inline uint32_t GPIO_STATE(GPIO_TypeDef* GPIOx, uint32_t PinMask)
{
	return LL_GPIO_IsInputPinSet(GPIOx, PinMask);
}

static inline void GPIO_SET(GPIO_TypeDef* GPIOx, uint32_t PinMask)
{
	LL_GPIO_SetOutputPin(GPIOx, PinMask);
}

static inline void GPIO_CLR(GPIO_TypeDef* GPIOx, uint32_t PinMask)
{
	LL_GPIO_ResetOutputPin(GPIOx, PinMask);
}

static inline void GPIO_TOGGLE(GPIO_TypeDef* GPIOx, uint32_t PinMask)
{
	LL_GPIO_TogglePin(GPIOx, PinMask);
}

static inline void GPIO_MODE(GPIO_TypeDef* GPIOx, uint32_t PinMask, uint32_t GPIO_Mode)
{
	LL_GPIO_SetPinMode(GPIOx, PinMask, GPIO_Mode);
}

static inline void GPIO_SPEED(GPIO_TypeDef* GPIOx, uint32_t PinMask, uint32_t GPIO_Speed)
{
	LL_GPIO_SetPinSpeed(GPIOx, PinMask, GPIO_Speed);
}

static inline void GPIO_OTYPE(GPIO_TypeDef* GPIOx, uint32_t PinMask, uint32_t GPIO_OType)
{
	LL_GPIO_SetPinOutputType(GPIOx, PinMask, GPIO_OType);
}

static inline void GPIO_PUPD(GPIO_TypeDef* GPIOx, uint32_t PinMask, uint32_t GPIO_PuPd)
{
	LL_GPIO_SetPinPull(GPIOx, PinMask, GPIO_PuPd);
}

#endif /* DRIVERS_GPIO_H_ */
