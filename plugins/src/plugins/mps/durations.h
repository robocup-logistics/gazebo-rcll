/***************************************************************************
 *  durations.h - MPS operation durations 
 *
 *  Created:   Tue 26 Oct 14:13:39 CEST 2021
 *  Copyright  2021  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
 ****************************************************************************/
/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.md file.
 */

#pragma once

#include <chrono>

namespace gazebo {
constexpr const std::chrono::milliseconds move_duration{2500};
constexpr const std::chrono::milliseconds dispense_duration{1500};
constexpr const std::chrono::milliseconds cap_op_duration{3500};
constexpr const std::chrono::milliseconds deliver_duration{3500};
constexpr const std::chrono::milliseconds ring_op_duration{3500};
} // namespace gazebo
