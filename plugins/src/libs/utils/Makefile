#*****************************************************************************
#              Makefile Build System for Fawkes: Utility Library
#                            -------------------
#   Created on Sun Sep 03 14:14:14 2006
#   Copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_libutils = stdc++ m rt core $(if $(filter-out Darwin,$(OS)),rt) $(if $(filter Linux,$(OS)),dl pthread)
OBJS_libutils = $(filter-out $(FILTER_OUT),$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp))))))
HDRS_libbutils = $(subst $(SRCDIR)/,,$(filter-out $(patsubst %.o,%.h,$(FILTER_OUT)),$(wildcard $(SRCDIR)/*.h $(SRCDIR)/*/*.h)))

OBJS_all = $(OBJS_libutils)
LIBS_all  = $(LIBDIR)/libutils.so

include $(BUILDSYSDIR)/base.mk

