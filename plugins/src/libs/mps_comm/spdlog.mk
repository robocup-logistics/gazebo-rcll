#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Mon 04 May 2020 10:30:53 CEST
#   Copyright (C) 2020 by Till Hofmann <hofmann@kbsg.rwth-aachen.de>
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

ifneq ($(PKGCONFIG),)
  HAVE_SPDLOG= $(if $(shell $(PKGCONFIG) --exists spdlog; echo $${?/1/}),1,0)
endif

ifeq ($(HAVE_SPDLOG),1)
  CFLAGS_SPDLOG = $(shell $(PKGCONFIG) --cflags spdlog)
  LDFLAGS_SPDLOG = $(shell $(PKGCONFIG) --libs spdlog)
endif
