#*****************************************************************************
#           Makefile Build System for Fawkes: Configuration Utils
#                            -------------------
#   Created on Sun Jan 07 14:56:02 2007
#   Copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
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

ifneq ($(HAVE_YAMLCPP),1)
  ERROR_TARGETS += error_yamlcpp
else
  CFLAGS  += $(CFLAGS_YAMLCPP)
  LDFLAGS += $(LDFLAGS_YAMLCPP)
  LIBS_all  = $(LIBDIR)/libconfig.so
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(ERROR_TARGETS)
.PHONY: error_yamlcpp
error_yamlcpp:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)LLSF RefBox config library cannot be built$(TNORMAL) (yaml-cpp not found)"
	$(SILENT)exit 2
endif

LIBS_libconfig = stdc++ core utils
OBJS_libconfig =$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))
HDRS_libconfig = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h))

OBJS_all = $(OBJS_libconfig)

include $(BUILDSYSDIR)/base.mk

