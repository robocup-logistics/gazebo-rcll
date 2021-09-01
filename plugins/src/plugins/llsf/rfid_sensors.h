/***************************************************************************
 *  rfid_sensors.h - checks if a puck is under a rfid sensor
 *
 *  Created: Mon Aug 26 19:21:03 2013
 *  Copyright  2013  Frederik Zwilling
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

#ifndef _RFID_SENSORS_HH_
#define _RFID_SENSORS_HH_

//radius of the area in which the sensor recognizes a puck
#define UNDER_RFID_TOL 0.05
//distance of the rfid-sensor-center from machine-center in x-dim
#define DIST_CENTER_RFID 0.06

#include "data_table.h"

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <string>

namespace gazebo {
/**
   * checks if a puck is under a rfid sensor
   *
   * (only works if you do not move the field)
   */
class RfidSensors
{
public:
	///Constructor
	RfidSensors();
	///Deconstructor
	~RfidSensors();

	///what to do on the plugin update (check for puck under rfid-sensors and write results to data table)
	void update();

private:
	///time variable to send in intervals
	double last_sent_time_;

	///Pointer to simulation data
	LlsfDataTable *table_;
};
} // namespace gazebo
#endif
