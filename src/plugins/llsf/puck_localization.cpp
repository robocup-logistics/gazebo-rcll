/***************************************************************************
 *  puck_localization.cpp - locates all pucks and writes position in table
 *
 *  Created: Mon Aug 26 20:27:45 2013
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

#include "puck_localization.h"
#include <stdio.h>


using namespace gazebo;

/** Constructor
 * @param world World, where to search for pucks
 */
PuckLocalization::PuckLocalization(physics::WorldPtr world)
{
  table_ = LlsfDataTable::get_table();
  world_ = world;
}

PuckLocalization::~PuckLocalization()
{

}

void PuckLocalization::update()
{
  for(int p = 0; p < NUMBER_PUCKS; p++)
  {
    Puck puck = table_->get_puck(p);

    //get position from world position of the Puck model
    if(!world_->GetEntity(puck.name_link.c_str()))
    {
      printf("Can not find puck with index %d\n", p);
      return;
    }
    double x = world_->GetEntity(puck.name_link.c_str())->GetWorldPose().pos.x;
    double y = world_->GetEntity(puck.name_link.c_str())->GetWorldPose().pos.y;

    //write it into the data table
    table_->set_puck_pos(p, x, y);
  }
}
