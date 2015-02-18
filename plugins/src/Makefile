#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Mon May 14 11:38:00 2012
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

BASEDIR = ..

SUBDIRS = libs plugins

include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/rules.mk

# Explicit dependencies, this is needed to have make bail out if there is any
# error. This is also necessary for working parallel build (i.e. for dual core)
plugins: libs
#tools: plugins

