/***************************************************************************
 *  refbox_comm.cpp - responsable for the communication with the refbox
 *                  reads/writes the data in the table
 *
 *  Created: Wed Aug 14 21:25:34 2013
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
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/transport/transport.hh>

#include "refbox_comm.h"
#include "data_table.h"

#include <llsf_msgs/MachineCommands.pb.h>
#include <llsf_msgs/MachineInfo.pb.h>
#include <llsf_msgs/PuckInfo.pb.h>

using namespace gazebo;

/** Constructor
 * @param table The data table to write the changes on
 * @param gazebo_node Transport node to publish and subscribe messages on
 */
RefboxComm::RefboxComm(LlsfDataTable *table, transport::NodePtr gazebo_node)
{
  table_ = table;
  gazebo_node_ = gazebo_node;

  //create publisher
  this->place_puck_under_machine_pub_ = gazebo_node_->Advertise<llsf_msgs::PlacePuckUnderMachine>("~/LLSFRbSim/PlacePuckUnderMachine/");
  this->remove_puck_from_machine_pub_ = gazebo_node_->Advertise<llsf_msgs::RemovePuckFromMachine>("~/LLSFRbSim/RemovePuckFromMachine/");

  //create subscriber
  this->machine_info_sub_ = gazebo_node_->Subscribe(std::string("~/LLSFRbSim/MachineInfo/"), &RefboxComm::on_machine_info_msg, this);
  this->puck_info_sub_ = gazebo_node_->Subscribe(std::string("~/LLSFRbSim/PuckInfo/"), &RefboxComm::on_puck_info_msg, this);
}

RefboxComm::~RefboxComm()
{
}

/** send protobuf msg for puck placed under rfid to fawkes and refbox
 * @param puck number of puck
 * @param machine machine
 */
void RefboxComm::send_puck_placed_under_rfid(int puck, Machine & machine)
{
  //create the message
  llsf_msgs::PlacePuckUnderMachine ppum;
  ppum.set_puck_id(puck+1);//+1 because the refbox starts with 1
  ppum.set_machine_name(machine.name_string);

  //publish
  place_puck_under_machine_pub_->Publish(ppum);
}
/** send protobuf msg for puck removed under rfid to fawkes and refbox
 * @param puck number of puck
 * @param machine machine
 */
void RefboxComm::send_remove_puck_from_machine(int puck, Machine & machine)
{
  //create the message
  llsf_msgs::RemovePuckFromMachine rpfm;
  rpfm.set_puck_id(puck+1);//+1 because the refbox starts with 1
  rpfm.set_machine_name(machine.name_string);

  //publish
  remove_puck_from_machine_pub_->Publish(rpfm);
}

void RefboxComm::on_machine_info_msg(ConstMachineInfoPtr &msg)
{
  //printf("Got MachineInfo :D\n");  
  //read all machines and set light signals
  for(int i = 0; i < msg->machines_size(); i++)
  {
    llsf_msgs::Machine machine = msg->machines(i);
    LightState red, yellow, green;
    //set default values
    red = OFF;
    yellow = OFF;
    green = OFF;
    for(int j = 0; j < machine.lights_size(); j++)
    {
      llsf_msgs::LightSpec light_spec = machine.lights(j);
      LightState state = OFF;
      switch(light_spec.state())
      {
      case llsf_msgs::OFF: state = OFF; break;
      case llsf_msgs::ON: state = ON; break;
      case llsf_msgs::BLINK: state = BLINK; break;
      }
      switch(light_spec.color())
      {
      case llsf_msgs::RED: red = state; break;
      case llsf_msgs::YELLOW: yellow = state; break;
      case llsf_msgs::GREEN: green = state; break;
      }
    }
    table_->set_light_state(machine.name(), red, yellow, green);
    if(machine.team_color() == llsf_msgs::CYAN)
    {
      table_->set_machine_team(machine.name(), CYAN);
    }
    if(machine.team_color() == llsf_msgs::MAGENTA)
    {
      table_->set_machine_team(machine.name(), MAGENTA);
    }
  }
}

void RefboxComm::on_puck_info_msg(ConstPuckInfoPtr &msg)
{
  //printf("Got PuckInfo\n");  
  for(int i = 0; i < msg->pucks_size(); i++)
  {
    llsf_msgs::Puck puck = msg->pucks(i);
    int id = puck.id() - 1;//-1 because the refbox starts with 1
    if(id < NUMBER_PUCKS)
    {
      table_->set_puck_state(id, puck.state());
    }
  }
}
