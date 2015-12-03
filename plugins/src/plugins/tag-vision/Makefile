#*****************************************************************************
#               Makefile Build System for Fawkes: Gazebo plugin tag-vision
#
#   Created on Fri Oct 16 16:53:24 2015
#   Copyright (C) 2015 by Frederik Zwilling
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

LIBS_gazebo_libtag_vision = gazsim_msgs llsf_msgs
OBJS_gazebo_libtag_vision = tag-vision.o

OBJS_all    = $(OBJS_gazebo_libtag_vision)

ifeq ($(HAVE_GAZEBO)$(HAVE_PROTOBUF)$(HAVE_CPP11),111)
  CFLAGS  += $(CFLAGS_CPP11) $(CFLAGS_GAZEBO) $(CFLAGS_PROTOBUF)
  LDFLAGS += $(LDFLAGS_GAZEBO) $(LDFLAGS_PROTOBUF) -lm $(call boost-libs-ldflags,system) -lboost_system

  LIBS_all = $(LIBDIR)/gazebo/libtag_vision.so
else
  ifneq ($(HAVE_GAZEBO),1)
    WARN_TARGETS += warning_gazebo
  endif
  ifneq ($(HAVE_PROTOBUF),1)
    WARN_TARGETS += warning_protobuf
  endif
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS = warning_cpp11
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_gazebo warning_protobuf
warning_gazebo:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazebo-tag_vision Plugin$(TNORMAL) " \
		"(Gazebo Simulator not found)"
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazebo-tag_vision Plugin$(TNORMAL) " \
		"(protobuf[-devel] not installed)"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazebo-tag_vision Plugin$(TNORMAL) (C++11 support required)"
endif

include $(BUILDSYSDIR)/base.mk
