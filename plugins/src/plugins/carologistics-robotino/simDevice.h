/***************************************************************************
 *  simDevice.h - general superclass for simulated devices
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

#ifndef __GAZEBO_SIM_DEVICE_H_
#define __GAZEBO_SIM_DEVICE_H_


#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/transport/transport.hh>

namespace gazebo
{
  /**
   * general superclass for simulated devices
   * @author Frederik Zwilling
   */
  class SimDevice
  {
  public:
    //Constructor
    SimDevice(physics::ModelPtr model, transport::NodePtr node);
    ///Destructor
    virtual ~SimDevice();
    //common functions
    
    /** Initialization
     */
    virtual void init() = 0;
    /** Creation of all publishers needed for the device
     */
    virtual void create_publishers() = 0;
    /** Creation of all subscribers needed for the device
     */
    virtual void create_subscribers() = 0;
    /** What to do in the update step of the plugin
     */
    virtual void update() = 0;


  protected:
    //Gazebo-objects needed by every device
    /// Pointer to the gazebo model
    physics::ModelPtr model;
    ///Node for communication
    transport::NodePtr node;
    ///time variable to send in intervals
    double last_sent_time_;
  };
}
#endif
