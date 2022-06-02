/***************************************************************************
 *  base_station.h - controls a basesation mps
 *
 *  Generated: Wed Apr 22 13:25:39 2015
 *  Copyright  2015  Randolph Maaßen
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef BASESTATION_H
#define BASESTATION_H

#include "mps.h"

namespace gazebo {

enum class BaseColor { RED = 1, SILVER, BLACK };

std::ostream&
operator<<(std::ostream& os, const BaseColor& b);

class BaseStation : public Mps
{
public:
	BaseStation(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

	void process_command_in() override;
	void dispense_base(BaseColor color);

private:
	void               on_new_puck(ConstNewPuckPtr &msg);
	std::string        spawning;
	gazsim_msgs::Color spawn_clr;
};

} // namespace gazebo
#endif // BASESTATION_H
