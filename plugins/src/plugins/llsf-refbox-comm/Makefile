#*****************************************************************************
#               Makefile Build System for Fawkes: llsf-refbox-comm
#                            -------------------
#   Created on Fri Mar 06 16:02:13 2015
#   Copyright (C) 2015 by Frederik Zwilling
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
include $(BUILDSYSDIR)/gazebo.mk
include $(BUILDSYSDIR)/protobuf.mk
include $(BUILDSYSDIR)/boost.mk


CFLAGS += $(CFLAGS_CPP11)

LIBS_gazebo_libllsf_refbox_comm = fawkes_protobuf_comm llsf_msgs gazsim_msgs
OBJS_gazebo_libllsf_refbox_comm = llsf_refbox_comm.o

ifeq ($(HAVE_GAZEBO_6),1)
  # workaround for missing libm in ignition-math pkgconfig file, cf.
  # https://bitbucket.org/ignitionrobotics/ign-math/issues/36/ignition-math2pc-should-specify-lm
  LIBS_gazebo_libllsf_refbox_comm += m
endif

OBJS_all = $(OBJS_gazebo_libllsf_refbox_comm)

REQ_BOOST_LIBS = system
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

ifeq ($(HAVE_GAZEBO)$(HAVE_PROTOBUF)$(HAVE_BOOST_LIBS),111)
  CFLAGS  += $(CFLAGS_PROTOBUF) \
	     $(call boost-libs-cflags,$(REQ_BOOST_LIBS)) \
	     $(CFLAGS_GAZEBO)
  LDFLAGS += $(LDFLAGS_PROTOBUF) \
	     $(call boost-libs-ldflags,$(REQ_BOOST_LIBS)) \
	     $(LDFLAGS_GAZEBO)

  LIBS_all = $(LIBDIR)/gazebo/libllsf_refbox_comm.so
else
  ifneq ($(HAVE_GAZEBO),1)
    WARN_TARGETS += warning_gazebo
  endif
  ifneq ($(HAVE_PROTOBUF),1)
    WARN_TARGETS += warning_protobuf
  endif
  ifneq ($(HAVE_BOOST_LIBS),1)
    WARN_TARGETS += warning_boost
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_protobuf warning_gazebo warning_boost
warning_gazebo:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting llsf-refbox-comm Plugin$(TNORMAL) " \
		"(Gazebo Simulator not found)"
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting llsf-refbox-comm Plugin$(TNORMAL) " \
		"(protobuf[-devel] not installed)"
warning_boost:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting llsf-refbox-comm Plugin$(TNORMAL) " \
		"(req. boost libs not found)"
endif


include $(BUILDSYSDIR)/base.mk

