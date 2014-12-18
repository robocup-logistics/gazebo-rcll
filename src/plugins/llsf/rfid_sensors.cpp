/***************************************************************************
 *  rfid_sensors.cpp - checks if a puck is under a rfid sensor
 *
 *  Created: Mon Aug 26 19:39:54 2013
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

#include "rfid_sensors.h"
#include <stdio.h>


using namespace gazebo;

RfidSensors::RfidSensors()
{
  table_ = LlsfDataTable::get_table();
}

RfidSensors::~RfidSensors()
{

}

void RfidSensors::update()
{ 
  //for all machines check if there is any puck anderneath it
  Machine* machines = table_->get_machines();
  for(int m = 0; m < NUMBER_MACHINES; m++)
  {
    Machine machine = machines[m];
    //calculate center of rfid-sensor
    double rfid_x = machine.x + DIST_CENTER_RFID * cos(machine.ori);
    double rfid_y = machine.y + DIST_CENTER_RFID * sin(machine.ori);
    
    //check all pucks
    for(int p = 0; p < NUMBER_PUCKS; p++)
    {
      Puck puck = table_->get_puck(p);
      double dist = sqrt((puck.x - rfid_x) * (puck.x - rfid_x)
			 + (puck.y - rfid_y) * (puck.y - rfid_y));
      if(dist < UNDER_RFID_TOL)
      {
	//printf("Puck %d is under %s\n", p, machine.name_link.c_str());
	if(puck.under_rfid != NONE && puck.under_rfid != machine.name)
	{
	  table_->remove_puck_under_rfid(p, puck.under_rfid);
	}
	table_->set_puck_under_rfid(p, machine.name);
      }
      else if (puck.under_rfid == machine.name)
      {
	//printf("Puck %d is no longer under %s\n", p, machine.name_link.c_str());
	table_->remove_puck_under_rfid(p, machine.name);
      }
    }
  }
}
