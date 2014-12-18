/***************************************************************************
 *  machineVision.h - provides ground truth about 
 *                    the nearest machine light signals
 *
 *  Created: Sat Aug 17 23:20:55 2013
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

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/transport/transport.hh>
#include "simDevice.h"
#include "../llsf/data_table.h"


namespace gazebo
{
  /**
   * This class simulates the results of the machine light signal detection
   */
  class MachineVision: public SimDevice
  {
  public:

    //Constructor
    MachineVision(physics::ModelPtr, transport::NodePtr);
    ///Destructor
    ~MachineVision();  

    virtual void init();
    virtual void create_publishers();
    virtual void create_subscribers();
    virtual void update();

  private:
    ///Functions for sending ionformation to fawkes:
    void send_light_results();

    ///Publisher for light results
    transport::PublisherPtr light_signal_pub_;

    ///Table with the simulation data
    LlsfDataTable *table_;

    void send_lights();
  };
}
