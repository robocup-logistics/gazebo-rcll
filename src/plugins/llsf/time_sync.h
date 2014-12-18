/***************************************************************************
 *  time_sync.h - syncs the simulation time with fawkes by sending
 *                the current real time factor
 *
 *  Created: Tue Sep 24 14:06:13 2013
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

#ifndef TIME_SYNC_H__
#define TIME_SYNC_H__

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include "../../msgs/SimTime.pb.h"


namespace gazebo
{
  /**
   *  syncs the simulation time with fawkes by sending the current real time factor
   *
   *  For comunication with the refbox, it uses the Protobuf-Adapter in Fawkes
   */
  class TimeSync
  {
  public:
    //Constructor
    TimeSync(physics::WorldPtr _world, transport::NodePtr gazebo_node);
    ///Destructor
    ~TimeSync();

    /// send protobuf msg with sim-time and real-time-factor
    void send_time_sync();

  private:
    ///Pointer to the communication node from gazebo
    transport::NodePtr gazebo_node_;
    ///World to get the time from
    physics::WorldPtr world_; 

    ///Publisher for communication
    transport::PublisherPtr time_sync_pub_;

    ///helper variables to calculate real time factor
    double last_real_time_;
    double last_sim_time_;
  };
}
#endif
