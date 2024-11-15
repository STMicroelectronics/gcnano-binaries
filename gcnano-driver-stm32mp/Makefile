##############################################################################
#
#    Copyright (C) 2005 - 2024 by Vivante Corp.
#
#    This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation; either version 2 of the license, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program; if not write to the Free Software
#    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
#
###############################################################################

AQROOT ?= $(CURDIR)
AQARCH := $(AQROOT)/arch/XAQ2

KERNEL_DIR        ?= /home/software/Linux/x86_pcie/linux-headers-4.8.0-41-generic
CROSS_COMPILE     ?=
ARCH_TYPE         ?= x86_64

include $(AQROOT)/config

export AQROOT AQARCH
export KERNEL_DIR CROSS_COMPILE

.PHONY: all clean install

all:
	@make -f Kbuild ARCH_TYPE=$(ARCH_TYPE)

clean:
	@make -f Kbuild clean

install:
	@make -f Kbuild ARCH_TYPE=$(ARCH_TYPE) install
