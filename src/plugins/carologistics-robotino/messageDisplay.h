/***************************************************************************
 *  messageDisplay.h - Shows text messages sent from fawkes
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

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/transport/transport.hh>
#include "simDevice.h"

namespace gazebo
{
  /**
   * Gazebo Plugin, which shows text messages sent from fawkes
   * @author Frederik Zwilling
   */
  class MessageDisplay: public SimDevice
  {
  public:

    //Constructor
    MessageDisplay(physics::ModelPtr, transport::NodePtr);
    ///Destructor
    ~MessageDisplay();

    virtual void init();
    virtual void create_publishers();
    virtual void create_subscribers();
    virtual void update();

  private:
    //Suscriber for Messages from Fawkes
    transport::SubscriberPtr string_sub_;
    
    //Functions for recieving Messages (registerd via suscribers)
    void on_string_msg(ConstHeaderPtr &msg);  
  };
}
