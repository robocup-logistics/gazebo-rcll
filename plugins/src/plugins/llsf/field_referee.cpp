/***************************************************************************
 *  field_referee.cpp - the field referee stands next to the field 
 *                      and takes out finished pucks
 *
 *  Created: Fri Sep 27 16:43:09 2013
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

#include "field_referee.h"
#include <stdio.h>


using namespace gazebo;

/** Constructor
 * @param world World, where the referee should operate
 */
FieldReferee::FieldReferee(physics::WorldPtr world)
{
  table_ = LlsfDataTable::get_table();
  world_ = world;
  
  finished_pucks_ = 0;
  waiting_before_removing_ = false;
  //this->sit_next_to_the_field_and_take_a_nap();
}

FieldReferee::~FieldReferee()
{

}

void FieldReferee::update()
{
  //check if the referee app says that there is a puck which has to be removed
  for(int p = 0; p < NUMBER_PUCKS; p++)
  {
    
    Puck puck = table_->get_puck(p);
    if(puck.state == llsf_msgs::FINISHED && puck.x < 5.6 && puck.y < 5.6)
    {
      if(waiting_before_removing_)
      {
	if(world_->GetSimTime().Double() > start_waiting_time_ + WAIT_TIME_BEFORE_REMOVE)
	{
	  //build a tower
	  math::Pose pose(6.0, 2.8, (0.1 + finished_pucks_ * 0.05), 0, 0, 0);
	  world_->GetEntity(puck.name_link.c_str())->SetWorldPose(pose, true, true);
	  finished_pucks_++;
	  waiting_before_removing_ = false;
	}
      }
      else
      {
	waiting_before_removing_ = true;
        start_waiting_time_ = world_->GetSimTime().Double();
      }
    }
  }
}
