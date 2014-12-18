/***************************************************************************
 *  data_table.cpp - This module stores all logical information
 *    about the llsf simulation (e.g. machine orientations,
 *    light signals, puck locations)
 *
 *  Created: Fri Aug 09 12:33:22 2013
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
#include <stddef.h>
#include <sstream>

#include "data_table.h"
#include "refbox_comm.h"


using namespace gazebo;

/** Constructor (Singleton)
 * @param world World, the data should be stored about
 * @param gazebo_node Transport node to publish and subscribe messages on
 */
LlsfDataTable::LlsfDataTable(physics::WorldPtr world, transport::NodePtr gazebo_node)
{
  world_ = world;

  //read out machine positions and orientation from world
  init_table();

  //initialize refbox communication
  refbox_comm_ = new RefboxComm(this, gazebo_node);
}

LlsfDataTable::~LlsfDataTable()
{
}

LlsfDataTable* LlsfDataTable::table_ = NULL;

/** Initialization of the data table
 * @param world World, the data should be stored about
 * @param gazebo_node Transport node to publish and subscribe messages on
 */
void LlsfDataTable::init(physics::WorldPtr world, transport::NodePtr gazebo_node)
{
  if(!table_)
  {
    table_ = new LlsfDataTable(world, gazebo_node);
  }
}

/** Getter for Singleton
 * @return pointer to singleton
 */
LlsfDataTable* LlsfDataTable::get_table()
{
  return table_;
}

/** Finalize Singleton
 */
void LlsfDataTable::finalize()
{
  if(table_)
  {
    delete table_;
  }
}

/** Getter for machine data
 * @param name Name of the machine (enum)
 * @return machine data
 */
Machine LlsfDataTable::get_machine(MachineName name)
{
  return machines_[name];
}

/** Getter for machine data
 * @param machine Name of the machine (string)
 * @return machine data
 */
Machine LlsfDataTable::get_machine(std::string machine)
{
  for(int i = M1; i <= R2; i++)
  {
    //is it the right machine?
    if(machines_[i].name_string.compare(machine) == 0)
    {
      return machines_[i];
    }
  }
  //default  
  return machines_[0];
}

/** Getter for machine data (all machines)
 * @return pointer to machine data
 */
Machine* LlsfDataTable::get_machines()
{
  return machines_;
}

/** Getter for puck data
 * @param number number of the puck
 * @return puck data
 */
Puck LlsfDataTable::get_puck(int number)
{
  return pucks_[number]; //assuming the pucks numbers start at 0
}


/** Setter for light state
 * @param machine Name of the machine (enum)
 * @param red state of red light
 * @param yellow state of yellow light
 * @param green state of green light
 */
void LlsfDataTable::set_light_state(MachineName machine, LightState red, 
				    LightState yellow, LightState green)
{
  machines_[machine].red = red;
  machines_[machine].yellow = yellow;
  machines_[machine].green = green;
}

/** Setter for light state
 * @param machine Name of the machine (string)
 * @param red state of red light
 * @param yellow state of yellow light
 * @param green state of green light
 */
void LlsfDataTable::set_light_state(std::string machine, LightState red,
		     LightState yellow, LightState green)
{
  for(int i = M1; i <= R2; i++)
  {
    //is it the right machine?
    if(machines_[i].name_string.find(machine) == 0)
    {
      set_light_state(machines_[i].name, red, yellow, green);
      return;
    }
  }
}

/** Setter for machine team
 * @param machine Name of the machine (string)
 * @param team name of the team
 */
void LlsfDataTable::set_machine_team(std::string machine, Team team)
{
  for(int i = M1; i <= R2; i++)
  {
    //is it the right machine?
    if(machines_[i].name_string.find(machine) == 0)
    {
      machines_[i].team = team;
      return;
    }
  }
}

/** Setter for puck position
 * @param puck Number of the puck
 * @param x x coordinate of position
 * @param y y coordinate of position
 */
void LlsfDataTable::set_puck_pos(int puck, double x, double y)
{
  pucks_[puck].x = x;
  pucks_[puck].y = y;
}

/** Mark a puck as placed under a machine
 * @param puck number of the puck
 * @param machine  name of the machine (enum)
 */
void LlsfDataTable::set_puck_under_rfid(int puck, MachineName machine)
{
  pucks_[puck].under_rfid = machine;
  //inform refbox
  refbox_comm_->send_puck_placed_under_rfid(puck, machines_[machine]);
}

/** Mark a puck as removed from a machine
 * @param puck number of the puck
 * @param machine  name of the machine (enum)
 */
void LlsfDataTable::remove_puck_under_rfid(int puck, MachineName machine)
{
  pucks_[puck].under_rfid = NONE;
  //inform refbox
  refbox_comm_->send_remove_puck_from_machine(puck, machines_[machine]);
}

/** Mark a puck as placed inside a machine-area
 * @param puck number of the puck
 * @param machine  name of the machine (enum)
 */
void LlsfDataTable::set_puck_in_machine_area(int puck, MachineName machine)
{
  pucks_[puck].in_machine_area = machine;
  //TODO: inform refbox if a puck leaves a machine area
}

/** Setter for puck state (S0,P1,FI,...)
 * @param puck number of the puck
 * @param state state of the puck
 */
void LlsfDataTable::set_puck_state(int puck, llsf_msgs::PuckState state)
{
  pucks_[puck].state = state;
}

void LlsfDataTable::init_table()
{
  init_machine(M1, "llsf_field::M1::machine_link", "M1");
  init_machine(M2, "llsf_field::M2::machine_link", "M2");
  init_machine(M3, "llsf_field::M3::machine_link", "M3");
  init_machine(M4, "llsf_field::M4::machine_link", "M4");
  init_machine(M5, "llsf_field::M5::machine_link", "M5");
  init_machine(M6, "llsf_field::M6::machine_link", "M6");
  init_machine(M7, "llsf_field::M7::machine_link", "M7");
  init_machine(M8, "llsf_field::M8::machine_link", "M8");
  init_machine(M9, "llsf_field::M9::machine_link", "M9");
  init_machine(M10, "llsf_field::M10::machine_link", "M10");
  init_machine(M11, "llsf_field::M11::machine_link", "M11");
  init_machine(M12, "llsf_field::M12::machine_link", "M12");
  init_machine(M13, "llsf_field::M13::machine_link", "M13");
  init_machine(M14, "llsf_field::M14::machine_link", "M14");
  init_machine(M15, "llsf_field::M15::machine_link", "M15");
  init_machine(M16, "llsf_field::M16::machine_link", "M16");
  init_machine(M17, "llsf_field::M17::machine_link", "M17");
  init_machine(M18, "llsf_field::M18::machine_link", "M18");
  init_machine(M19, "llsf_field::M19::machine_link", "M19");
  init_machine(M20, "llsf_field::M20::machine_link", "M20");
  init_machine(M21, "llsf_field::M21::machine_link", "M21");
  init_machine(M22, "llsf_field::M22::machine_link", "M22");
  init_machine(M23, "llsf_field::M23::machine_link", "M23");
  init_machine(M24, "llsf_field::M24::machine_link", "M24");
  init_machine(D1, "llsf_field::D1::machine_link", "D1");
  init_machine(D2, "llsf_field::D2::machine_link", "D2");
  init_machine(D3, "llsf_field::D3::machine_link", "D3");
  init_machine(D4, "llsf_field::D4::machine_link", "D4");
  init_machine(D5, "llsf_field::D5::machine_link", "D5");
  init_machine(D6, "llsf_field::D6::machine_link", "D6");
  init_machine(R1, "llsf_field::R1::machine_link", "R1");
  init_machine(R2, "llsf_field::R2::machine_link", "R2");

  for(int i = 0; i < NUMBER_PUCKS; i++)
  {
    std::ostringstream ss;
    ss << "Puck" << i << "::cylinder";
    init_puck(i, ss.str().c_str());
  }
}

void LlsfDataTable::init_machine(MachineName number, std::string name_link, std::string name_string)
{
  machines_[number].name = number;
  machines_[number].name_link = name_link;
  machines_[number].name_string = name_string;
  if(!world_->GetEntity(name_link))
  {
    printf("Can not find machine %s\n", name_link.c_str());
    return;
  }
  machines_[number].x = world_->GetEntity(name_link)->GetWorldPose().pos.x;
  machines_[number].y = world_->GetEntity(name_link)->GetWorldPose().pos.y;
  machines_[number].ori = world_->GetEntity(name_link)->GetWorldPose().rot.GetAsEuler().z;
  machines_[number].red = BLINK;//OFF;
  machines_[number].yellow = BLINK;//OFF;
  machines_[number].green = BLINK;//OFF;
  machines_[number].team = NIL;//OFF;
}

void LlsfDataTable::init_puck(int number, std::string name)
{
  pucks_[number].number = number;
  pucks_[number].name_link = name;
  pucks_[number].under_rfid = NONE;
  pucks_[number].in_machine_area = NONE;
  pucks_[number].state = llsf_msgs::S0;
}
