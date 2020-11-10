#*****************************************************************************
#                  Makefile Build System for the RCLL Refbox
#                            -------------------
#   Created on Sun 28 Apr 2019 16:00:20 CEST
#   Copyright (C) 2019 by Till Hofmann <hofmann@kbsg.rwth-aachen.de>
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

include $(BASEDIR)/src/libs/mps_comm/spdlog.mk

FREEOPCUA_COMPONENTS=libopcuacore libopcuaclient libopcuaprotocol libopcuaserver
ifneq ($(PKGCONFIG),)
  HAVE_FREEOPCUA= $(if $(shell $(PKGCONFIG) --exists $(FREEOPCUA_COMPONENTS); echo $${?/1/}),1,0)
endif

ifeq ($(HAVE_FREEOPCUA),1)
  CFLAGS_FREEOPCUA  = $(shell $(PKGCONFIG) --cflags $(FREEOPCUA_COMPONENTS)) $(CFLAGS_CPP11)
  LDFLAGS_FREEOPCUA = $(shell $(PKGCONFIG) --libs $(FREEOPCUA_COMPONENTS)) -lmbedtls
  # Check if we need to use the system spdlog
  ifneq ($(filter -DHAVE_SYSTEM_SPDLOG,$(CFLAGS_FREEOPCUA)),)
    # freeopcua uses the system spdlog, check if it is actually available
    ifeq ($(HAVE_SPDLOG),1)
      CFLAGS_FREEOPCUA += $(CFLAGS_SPDLOG)
      LDFLAGS_FREEOPCUA += $(LDFLAGS_SPDLOG)
    else
      HAVE_FREEOPCUA=0
      WARNING_FREEOPCUA=spdlog[-devel] missing
    endif
  endif
else
  WARNING_FREEOPCUA=freeopcua[-devel] not found
endif
