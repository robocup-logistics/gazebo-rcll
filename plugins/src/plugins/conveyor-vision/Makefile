#*****************************************************************************
#               Makefile Build System for Fawkes: Gazebo plugin for conveyor-vision
#
#   Created on Tue Jan 28 13:30:39 2014
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
#                         Frederik Zwilling
#
##*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../..

include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/gazebo.mk
include $(BUILDSYSDIR)/protobuf.mk

GAZEBO_LIBDIR = $(LIBDIR)/gazebo
LIBDIRS_BASE += $(GAZEBO_LIBDIR)

LIBS_gazebo_libconveyor_vision = gazsim_msgs llsf_msgs
OBJS_gazebo_libconveyor_vision = conveyor_vision.o

OBJS_all    = $(OBJS_gazebo_libconveyor_vision)

ifeq ($(HAVE_GAZEBO)$(HAVE_PROTOBUF)$(HAVE_CPP11),111)
  CFLAGS  += $(CFLAGS_GAZEBO) $(CFLAGS_PROTOBUF) $(CFLAGS_CPP11)
  LDFLAGS += $(LDFLAGS_GAZEBO) $(LDFLAGS_PROTOBUF) -lm $(call boost-libs-ldflags,system) -lboost_system

  LIBS_all = $(LIBDIR)/gazebo/libconveyor_vision.so
else
  ifneq ($(HAVE_GAZEBO),1)
    WARN_TARGETS += warning_gazebo
  endif
  ifneq ($(HAVE_PROTOBUF),1)
    WARN_TARGETS += warning_protobuf
  endif
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_gazebo warning_protobuf warning_cpp11
warning_gazebo:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazebo-conveyor-vision Plugin$(TNORMAL) " \
		"(Gazebo Simulator not found)"
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazebo-conveyor-vision Plugin$(TNORMAL) " \
		"(protobuf[-devel] not installed)"
waring_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazebo-conveyor-vision Plugin$(TNORMAL) " \
    "(cpp11 not available)"
endif

include $(BUILDSYSDIR)/base.mk
