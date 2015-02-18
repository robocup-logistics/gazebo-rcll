/***************************************************************************
 *  puck_localization.h - locates all pucks and writes position in table
 *
 *  Created: Mon Aug 26 20:25:44 2013
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

#ifndef _PUCK_LOCALIZATION_HH_
#define _PUCK_LOCALIZATION_HH_

#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "data_table.h"

namespace gazebo
{
 /**
   * locates all pucks and writes position in table
   */
  class PuckLocalization
  {
  public: 
    //Constructor
    PuckLocalization(physics::WorldPtr world);
    ///Deconstructor
    ~PuckLocalization();

    ///what to do on plugin update (write puck positions into data table)
    void update();

  private:
    ///Pointer to simulation data
    LlsfDataTable *table_;
    
    physics::WorldPtr world_;
  };
}
#endif
