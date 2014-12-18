/***************************************************************************
 *  light_control.h - Module to control the Machine Lights in the visualization
 *
 *  Created: Mon Aug 26 11:27:51 2013
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

#ifndef _LIGHT_CONTROL_HH_
#define _LIGHT_CONTROL_HH_

#include <string>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include "data_table.h"

namespace gazebo
{  
  typedef enum Color
  {
    RED,
    YELLOW,
    GREEN
  } Color;


  namespace msgs
  {
    class Visual;
  }

  /**
   * controls the Machine Lights in the visualization
   */
  class LightControl
  {
  public: 
    //Constructor
    LightControl(physics::WorldPtr world);
    ///Deconstructor
    virtual ~LightControl();
    
    /// what to do on plugin update (set lights according to the data table)
    void update();

  private:
    ///communication node
    transport::NodePtr node_;
    ///Publisher to send visual changes to gazebo
    transport::PublisherPtr visPub_;

    msgs::Visual create_vis_msg(std::string machine_name, Color color, LightState state);

    ///time variable to send in intervals
    double last_sent_time_;

    physics::WorldPtr world_;

    LlsfDataTable *table_;
  };
}
#endif
