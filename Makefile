##
## This file is part of the libopencm3 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##

DEVICE = stm32l011f3
OPENCM3_DIR = libopencm3

BINARY = vf-badge
OBJS += src/apa102.o
OBJS += src/at42qt1070.o
OBJS += src/leds.o
OBJS += src/schedule.o
CFLAGS += -I src -I
DEFS += -Wno-unused-parameter

include rules.mk

## Phony rule to ensure the target library gets built.
clean: libclean
.PHONY: $(OPENCM3_DIR) libclean
$(OBJS): $(OPENCM3_DIR)
$(OPENCM3_DIR):
	make -C $(OPENCM3_DIR) TARGETS=stm32/l0

libclean:
	make -C $(OPENCM3_DIR) clean

