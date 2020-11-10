#*****************************************************************************
#           Makefile Build System for Fawkes: mps_comm Library
#                            -------------------
#   Created on Thu Feb 14 09:30:32 2013
#   Copyright (C) 2013 by Tim Niemueller, AllemaniACs RoboCup Team
#                 2019 by Till Hofmann
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/protobuf.mk
include $(BUILDSYSDIR)/boost.mk
include $(BASEDIR)/src/libs/mps_comm/freeopcua.mk

LIBS_libmps_comm = stdc++ m pthread
#LIBS_libmps_comm = stdc++ m llsfrbcore llsfrbconfig pthread

OBJS_opcua = mps_server.o
#OBJS_opcua = opcua/opc_utils.o opcua/machine.o opcua/base_station.o \
						 opcua/cap_station.o opcua/delivery_station.o opcua/ring_station.o \
             opcua/storage_station.o
#OBJS_mockup = mockup/machine.o mockup/base_station.o mockup/cap_station.o \
							mockup/delivery_station.o mockup/ring_station.o \
              mockup/storage_station.o
#OBJS_libmps_comm = time_utils.o machine_factory.o

ifeq ($(HAVE_CPP17),1)
  OBJS_libmps_comm += $(OBJS_mockup)
  CFLAGS += $(CFLAGS_CPP17) -DHAVE_MOCKUP
else
  WARN_TARGETS += warning_cpp17
endif

ifeq ($(HAVE_FREEOPCUA),1)
  OBJS_libmps_comm += $(OBJS_opcua)
  CFLAGS  += $(CFLAGS_FREEOPCUA) -DHAVE_FREEOPCUA
  LDFLAGS += $(LDFLAGS_FREEOPCUA)
else
  WARN_TARGETS += warning_freeopcua
endif

OBJS_all = $(OBJS_libmps_comm)
LIBS_all = $(LIBDIR)/libmps_comm.so

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: $(WARN_TARGETS)
warning_freeopcua:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)No OPC-UA support$(TNORMAL) ($(WARNING_FREEOPCUA))"
warning_cpp17:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)No Mockup support$(TNORMAL) (C++17 not supported)"
endif

include $(BUILDSYSDIR)/base.mk
