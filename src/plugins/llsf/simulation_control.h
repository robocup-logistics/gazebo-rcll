/***************************************************************************
 *  simulation_control.h - shuts down the simulation on request
 *
 *  Created: Mon Oct 07 11:36:04 2013
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

#ifndef SIMULATION_CONTROL_H__
#define SIMULATION_CONTROL_H__

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>


namespace gazebo
{
  /**
   *  shuts down the simulation on request
   */
  class SimulationControl
  {
  public:
    //Constructor
    SimulationControl(physics::WorldPtr _world, transport::NodePtr gazebo_node);
    ///Destructor
    ~SimulationControl();

  private:
    ///Pointer to the communication node from gazebo
    transport::NodePtr gazebo_node_;
    ///World to get the time from
    physics::WorldPtr world_; 

    ///Publisher for communication
    transport::SubscriberPtr simulation_control_sub_;

    ///msg handler
    void on_string_msg(ConstHeaderPtr &msg);  
  };
}
#endif
