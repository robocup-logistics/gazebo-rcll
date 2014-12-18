/***************************************************************************
 *  puck_holder.h - holds a puck in the robotinos gripper while turning
 *
 *  Created: Mon Sep 30 16:57:47 2013
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

#ifndef _PUCK_HOLDER_HH_
#define _PUCK_HOLDER_HH_

#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "simDevice.h"
#include "../llsf/data_table.h"


namespace gazebo
{
  /**
   * holds a puck in the robotinos gripper while turning
   * Workaround for pucks slipping out of the gripper
   * can be turned off in the config
   */
  class PuckHolder: public SimDevice
  {
  public: 
    //Constructor
    PuckHolder(physics::ModelPtr, transport::NodePtr);
    ///Deconstructor
    ~PuckHolder();

    virtual void init();
    virtual void create_publishers();
    virtual void create_subscribers();
    virtual void update();


  private:
    ///Pointer to simulation data
    LlsfDataTable *table_;

    bool puck_attached_;
    physics::ModelPtr puck_model_;
  };
}
#endif
