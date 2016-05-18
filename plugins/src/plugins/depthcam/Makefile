#*****************************************************************************
#               Makefile Build System for Fawkes: Gazebo plugin depthcam
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
LIBDIRS_BASE += $(GAZEBO_LIBDIR) $(GAZEBO_RCLL)/plugins/lib

LIBS_gazebo_libdepthcam = configurable
OBJS_gazebo_libdepthcam = depthcam.o

OBJS_all    = $(OBJS_gazebo_libdepthcam)

ifneq ($(PKGCONFIG),)
  HAVE_OGRE = $(if $(shell $(PKGCONFIG) --exists 'OGRE'; echo $${?/1/}),1,0)
  ifeq ($(HAVE_OGRE),1)
    CFLAGS_OGRE  = -DHAVE_OGRE $(shell $(PKGCONFIG) --cflags 'OGRE')
    LDFLAGS_OGRE = $(shell $(PKGCONFIG) --libs 'OGRE')
  endif
  HAVE_OGRE_PAGING = $(if $(shell $(PKGCONFIG) --exists 'OGRE-Paging'; echo $${?/1/}),1,0)
  ifeq ($(HAVE_OGRE_PAGING),1)
    CFLAGS_OGRE_PAGING  = -DHAVE_OGRE_PAGING $(shell $(PKGCONFIG) --cflags 'OGRE-Paging')
    LDFLAGS_OGRE_PAGING = $(shell $(PKGCONFIG) --libs 'OGRE-Paging')
  endif
endif

ifeq ($(HAVE_GAZEBO)$(HAVE_PROTOBUF)$(HAVE_OGRE)$(HAVE_OGRE_PAGING),1111)
  CFLAGS  += $(CFLAGS_GAZEBO) $(CFLAGS_PROTOBUF) $(CFLAGS_OGRE) $(CFLAGS_OGRE_PAGING)
  LDFLAGS += $(LDFLAGS_GAZEBO) $(LDFLAGS_PROTOBUF) $(LDFLAGS_OGRE) $(LDFLAGS_OGRE_PAGING) \
             -lm $(call boost-libs-ldflags,system)

  LIBS_all = $(LIBDIR)/gazebo/libdepthcam.so
else
  ifneq ($(HAVE_GAZEBO),1)
    WARN_TARGETS += warning_gazebo
  endif
  ifneq ($(HAVE_PROTOBUF),1)
    WARN_TARGETS += warning_protobuf
  endif
  ifneq ($(HAVE_OGRE),1)
    WARN_TARGETS += warning_ogre
  endif
  ifneq ($(HAVE_OGRE_PAGING),1)
    WARN_TARGETS += warning_ogre_paging
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_gazebo warning_protobuf warning_ogre warning_ogre_paging
warning_gazebo:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazebo-depthcam Plugin$(TNORMAL) " \
		"(Gazebo Simulator not found)"
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazebo-depthcam Plugin$(TNORMAL) " \
		"(protobuf[-devel] not installed)"
warning_ogre:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazebo-depthcam Plugin$(TNORMAL) " \
		"(OGRE not found)"
warning_ogre_paging:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazebo-depthcam Plugin$(TNORMAL) " \
		"(OGRE Paging not found)"
endif

include $(BUILDSYSDIR)/base.mk
