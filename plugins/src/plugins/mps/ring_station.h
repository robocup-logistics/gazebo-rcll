/***************************************************************************
 *  ring_station.h - controls a ring station mps
 *
 *  Generated: Wed Apr 22 13:48:39 2015
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

#ifndef RING_STATION_H
#define RING_STATION_H

#include "mps.h"

namespace gazebo {

class RingStation : public Mps
{
public:
	RingStation(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

	void           process_command_in() override;
	gzwrap::Pose3d add_base_pose();

	void publish_indicator(bool active, int number);
	/// Handler for puck positions
	void on_puck_msg(ConstPosePtr &msg) override;

protected:
	bool puck_on_slide(const gzwrap::Pose3d &pose);
	bool puck_on_slide(ConstPosePtr &pose);

private:
	void                  mount_ring(gazsim_msgs::Color);
	std::set<std::string> wps_on_slide_;
};

} // namespace gazebo

#endif // RING_STATION_H
