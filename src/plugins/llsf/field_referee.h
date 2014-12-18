/***************************************************************************
 *  field_referee.h - the field referee stands next to the field and
 *                    takes out finished pucks
 *
 *  Created: Fri Sep 27 16:41:07 2013
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

#ifndef _FIELD_REFEREE_HH_
#define _FIELD_REFEREE_HH_

#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "data_table.h"

#define WAIT_TIME_BEFORE_REMOVE 3.0

namespace gazebo
{
  /**
   * the field referee statnds next to the field and takes
   * finished pucks out
   */
  class FieldReferee
  {
  public: 
    //Constructor
    FieldReferee(physics::WorldPtr world);
    ///Deconstructor
    ~FieldReferee();

    ///what to do on plugin update (move finished pucks outside the field)
    void update();

  private:
    ///Pointer to simulation data
    LlsfDataTable *table_;
    
    physics::WorldPtr world_;

    int finished_pucks_;
    
    bool waiting_before_removing_;
    double start_waiting_time_;
  };
}
#endif
