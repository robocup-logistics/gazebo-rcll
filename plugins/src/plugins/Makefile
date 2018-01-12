#*****************************************************************************
#                  Makefile Build System for Fawkes: Plugins
#                            -------------------
#   Created on Mon May 14 11:37:00 2012
#   Copyright (C) 2006-2012 by Tim Niemueller, Carologistics RoboCup Team
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

SUBDIRS	= gyro motor localization odometry \
	  mps light-control time-sync puck gripper llsf-refbox-comm \
	  light-signal-detection mps-placement conveyor-vision \
	  tag tag-vision depthcam floor-clean

include $(BUILDSYSDIR)/rules.mk


# Explicit dependencies
carologistics-robotino: llsf
conveyor-vision: mps
