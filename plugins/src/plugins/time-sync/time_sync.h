/***************************************************************************
 *  time_sync.h - The main plugin for synchronizing the time with a robot control software
 *
 *  Created: Mon Mar 02 15:54:47 2015
 *  Copyright  2015  Frederik Zwilling
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
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <gazsim_msgs/SimTime.pb.h>

#include <gazebo/gazebo.hh>

namespace gazebo {
/**
   * Main plugin for synchronizing the time with a robot control software
   */
class TimesyncPlugin : public WorldPlugin
{
public:
	///Constructor
	TimesyncPlugin();
	///Destructor
	~TimesyncPlugin();

	virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

private:
	///update function
	void                 Update();
	event::ConnectionPtr update_connection_;

	///Node for communication
	transport::NodePtr node_;

	physics::WorldPtr world_;

	double time_sync_frequency_;
	double last_time_sync_;

	///Publisher for communication
	transport::PublisherPtr time_sync_pub_;

	///helper variables to calculate real time factor
	double last_real_time_;
	double last_sim_time_;

	/// send protobuf msg with sim-time and real-time-factor
	void send_time_sync();
};
GZ_REGISTER_WORLD_PLUGIN(TimesyncPlugin)
} // namespace gazebo
