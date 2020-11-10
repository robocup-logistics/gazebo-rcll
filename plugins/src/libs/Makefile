#*****************************************************************************
#                  Makefile Build System for Fawkes: Libraries
#                            -------------------
#   Created on Tue Nov 07 16:12:59 2006
#   copyright (C) 2006-2011 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../..
include $(BASEDIR)/etc/buildsys/config.mk

SUBDIRS = gazsim_msgs llsf_msgs protobuf_comm \
	  core utils config configurable mps_comm

# Explicit dependencies, this is needed to have make bail out if there is any
# error. This is also necessary for working parallel build (i.e. for dual core)
all:
utils: core
config: core utils
configurable: config


include $(BUILDSYSDIR)/rules.mk


