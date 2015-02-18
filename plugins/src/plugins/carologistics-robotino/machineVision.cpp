/***************************************************************************
 *  machineVision.cpp - provides ground truth about 
 *                    the nearest machine light signals
 *
 *  Created: Sat Aug 17 23:24:45 2013
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
#include "machineVision.h"
#include "../llsf/data_table.h"
#include <llsf_msgs/LightSignals.pb.h>
#include <llsf_msgs/MachineInfo.pb.h>
#include "config.h"

using namespace gazebo;

/** Constructor
 * @param model Model to the gazebo model of the plugin
 * @param node Transport node to publish and subscribe messages on
 */
MachineVision::MachineVision(physics::ModelPtr model, transport::NodePtr node)
 : SimDevice(model, node)
{
}
MachineVision::~MachineVision()
{
}

void MachineVision::init()
{
  printf("Initialize MachineVision \n");
  table_ = LlsfDataTable::get_table();
}

void MachineVision::create_publishers()
{
  this->light_signal_pub_ = this->node->Advertise<llsf_msgs::AllMachineSignals>("~/RobotinoSim/MachineVision/");
}

void MachineVision::create_subscribers()
{
  //no subscribers
}

void MachineVision::update()
{
  //Send position information to Fawkes
  double time = model->GetWorld()->GetSimTime().Double();
  if(time - last_sent_time_ > (1.0 / LIGHT_SIGNAL_SEND_FREQUENCY))
  {
    last_sent_time_ = time;
    send_lights();
  }
}

void MachineVision::send_lights()
{
  //just send ground truth machine light signals with position to fawkes
  //the fawkes plugin takes care of the selection of the right machine in front of the robotino
  //because it knows which position the laser cluster has chosen
  
  //build Protobuf Message
  llsf_msgs::AllMachineSignals all_machines;
  for(int i = M1; i != R2; i++)
  {
    llsf_msgs::MachineSignal *machine_signal = all_machines.add_machines();
    //get mchine data
    Machine machine = table_->get_machine((MachineName) i);
    //set name
    machine_signal->set_name(machine.name_link);
    //set lights
    llsf_msgs::LightSpec *red = machine_signal->add_lights();
    red->set_color(llsf_msgs::RED);
    switch(machine.red)
    {
    case ON: red->set_state(llsf_msgs::ON); break;
    case BLINK: red->set_state(llsf_msgs::BLINK); break;
    default: red->set_state(llsf_msgs::OFF); break;
    }
    llsf_msgs::LightSpec *yellow = machine_signal->add_lights();
    yellow->set_color(llsf_msgs::YELLOW);
    switch(machine.yellow)
    {
    case ON: yellow->set_state(llsf_msgs::ON); break;
    case BLINK: yellow->set_state(llsf_msgs::BLINK); break;
    default: yellow->set_state(llsf_msgs::OFF); break;
    }
    llsf_msgs::LightSpec *green = machine_signal->add_lights();
    green->set_color(llsf_msgs::GREEN);
    switch(machine.green)
    {
    case ON: green->set_state(llsf_msgs::ON); break;
    case BLINK: green->set_state(llsf_msgs::BLINK); break;
    default: green->set_state(llsf_msgs::OFF); break;
    }
    //set position
    machine_signal->mutable_pose()->set_x(machine.x);
    machine_signal->mutable_pose()->set_y(machine.y);
    machine_signal->mutable_pose()->set_ori(machine.ori);
    //set timestamp of position (only needed to successfully compile)
    machine_signal->mutable_pose()->mutable_timestamp()->set_sec(0);
    machine_signal->mutable_pose()->mutable_timestamp()->set_nsec(0);
  }
  //send it
  light_signal_pub_->Publish(all_machines);
}
