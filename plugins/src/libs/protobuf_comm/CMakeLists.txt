# ***************************************************************************
# Created:   Mon  3 May 14:50:32 CEST 2021
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

find_package(PkgConfig REQUIRED)
pkg_check_modules(libcrypto IMPORTED_TARGET libcrypto)
if(NOT libcrypto_FOUND)
  pkg_check_modules(libcrypto IMPORTED_TARGET openssl)
endif()

add_library(protobuf_comm SHARED client.cpp crypto.cpp message_register.cpp
                                 peer.cpp server.cpp)

if(libcrypto_FOUND)
  target_link_libraries(protobuf_comm PkgConfig::libcrypto)
  target_compile_options(protobuf_comm PRIVATE "-DHAVE_LIBCRYPTO")
endif()
