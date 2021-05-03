# ***************************************************************************
# Created:   Mon  3 May 14:09:42 CEST 2021
#
# Copyright  2021  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
# ****************************************************************************/
# This program is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation; either version 2 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU Library General Public License for more
# details.
#
# Read the full text in the LICENSE.md file.
#

add_library(
  core SHARED
  exception.cpp
  plugin.cpp
  exceptions/software.cpp
  exceptions/system.cpp
  threading/barrier.cpp
  threading/interruptible_barrier.cpp
  threading/mutex.cpp
  threading/mutex_locker.cpp
  threading/read_write_lock.cpp
  threading/recursive_mutex.cpp
  threading/refc_rwlock.cpp
  threading/scoped_rwlock.cpp
  threading/spinlock.cpp
  threading/thread_collector.cpp
  threading/thread.cpp
  threading/thread_finalizer.cpp
  threading/thread_initializer.cpp
  threading/thread_list.cpp
  threading/thread_notification_listener.cpp
  threading/wait_condition.cpp
  utils/refcount.cpp)
