#*******************************************************************************
#
# Copyright (c) 2019, Vitaliy Dudnik <masterv.play@gmail.com>
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#   1. Redistributions of source code must retain the above copyright notice,
#      this list of conditions and the following disclaimer.
#   2. Redistributions in binary form must reproduce the above copyright notice,
#      this list of conditions and the following disclaimer in the documentation
#      and/or other materials provided with the distribution.
#   3. Neither the name of copyright holder nor the names of its contributors
#      may be used to endorse or promote products derived from this software
#      without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#*******************************************************************************

PART     = STM32F103xB
CPU      = cortex-m3

PROJECT = pt_example
TARGETS = $(BINDIR)/$(PROJECT).elf \
          $(BINDIR)/$(PROJECT).hex

#
# Include the common make definitions
#
include makedefs

#
# Include path
#
IPATH   = .                 \
          drivers           \
          ll                \
          ll/cmsis          \
          ll/cmsis/device   \
          mgmt              \
          os

#
# Source path
#
VPATH   = .                 \
          drivers           \
          ll                \
          mgmt              \
          os

#*******************************************************************************
#
# Main application
#
#*******************************************************************************
size: $(BINDIR)/$(PROJECT).elf

#
# LL section
#
#
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/stm32f1xx_ll_rcc.o
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/stm32f1xx_ll_rtc.o
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/stm32f1xx_ll_utils.o
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/stm32f1xx_ll_gpio.o
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/stm32f1xx_ll_i2c.o
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/stm32f1xx_ll_spi.o
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/stm32f1xx_ll_tim.o
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/stm32f1xx_ll_adc.o
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/stm32f1xx_ll_exti.o
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/stm32f1xx_ll_dma.o
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/stm32f1xx_ll_usart.o

#
# Driver section
#
#
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/button.o
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/uart3.o
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/i2c.o
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/spi_flash.o

#
# OS section
#
#
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/assert.o
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/pt.o
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/queue.o
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/timer.o
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/heap.o

#
# Mgmt section
#
#
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/sys_mgr.o
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/pwr_mgr.o
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/example_mgr.o

#
# Main section
#
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/main.o
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/system.o
$(BINDIR)/$(PROJECT).elf: ${OBJDIR}/startup.o
$(BINDIR)/$(PROJECT).elf: scatter.ld