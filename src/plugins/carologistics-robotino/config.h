/***************************************************************************
 *  config.h - config values for gazebo plugins
 *
 *  Created: Mon Jul 29 17:33:31 2013
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

///Difference in the position needed before gazebo sends a message to fawkes
#define POSITION_SEND_TOLLERANCE 0.001

/// Frequencies in hz
#define GPS_SEND_FREQUENCY 5.0
#define LIGHT_SIGNAL_SEND_FREQUENCY 2.0
///laser send frequency defined in hokuyo/model.sdf
#define PUCK_DETECTION_SEND_FREQUENCY 3.0
#define FRONT_CAMERA_SEND_FREQUENCY 2.0

#define ATTACH_PUCK_TO_GRIPPER_WHEN_TURNING true
#define DISTANCE_GRIPPER_CENTER_ROBOTINO 0.23
