/***************************************************************************
 *  mps.cpp - Plugin to control a simulated MPS
 *
 *  Created: Fri Feb 20 17:15:54 2015
 *  Copyright  2015  Frederik Zwilling
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

#include <math.h>

#include "mps.h"

using namespace gazebo;

// Register this plugin to make it available in the simulator
GZ_REGISTER_MODEL_PLUGIN(Mps)

///Constructor
Mps::Mps()
{
}
///Destructor
Mps::~Mps()
{
  printf("Destructing Mps Plugin!\n");
}

/** on loading of the plugin
 * @param _parent Parent Model
 */
void Mps::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) 
{
  // Store the pointer to the model
  this->model_ = _parent;

  //get the model-name
  this->name_ = model_->GetName();
  printf("Loading Mps Plugin of model %s\n", name_.c_str());

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&Mps::OnUpdate, this, _1));

  //Create the communication Node for communication with fawkes
  this->node_ = transport::NodePtr(new transport::Node());
  //the namespace is set to the world name!
  this->node_->Init(model_->GetWorld()->GetName());

  //subscribe for puck locations
  for(int i = 0; i < NUMBER_PUCKS; i++)
  {
    this->puck_subs_[i] = this->node_->Subscribe(std::string("~/puck_") + std::to_string(i) + "/gazsim/gps/" , &Mps::on_puck_msg, this);
  }

  //compute locations of input and output (not sure about the sides jet)
  double mps_x = this->model_->GetWorldPose().pos.x;
  double mps_y = this->model_->GetWorldPose().pos.y;
  double mps_ori = this->model_->GetWorldPose().rot.GetAsEuler().z;
  input_x_ = mps_x
    + BELT_OFFSET_SIDE  * cos(mps_ori)
    + (BELT_LENGTH / 2 - PUCK_SIZE) * sin(mps_ori);
  input_y_ = mps_y
    + BELT_OFFSET_SIDE  * sin(mps_ori)
    - (BELT_LENGTH / 2 - PUCK_SIZE) * cos(mps_ori);
  output_x_ = mps_x
    + BELT_OFFSET_SIDE  * cos(mps_ori)
    - (BELT_LENGTH / 2 - PUCK_SIZE) * sin(mps_ori);
  output_y_ = mps_y
    + BELT_OFFSET_SIDE  * sin(mps_ori)
    + (BELT_LENGTH / 2 - PUCK_SIZE) * cos(mps_ori);
  
  //set the machine type
  printf("detected machine type: ");
  if(this->name_.find("BS")!=std::string::npos)
  {
    this->machine_type_=MachineType::Base;
    printf("base");
  }
  else if(this->name_.find("CS")!=std::string::npos)
  {
    this->machine_type_=MachineType::Cap;
    printf("cap");
  }
  else if(this->name_.find("RS")!=std::string::npos)
  {
    this->machine_type_=MachineType::Ring;
    printf("ring");
  }
  else if(this->name_.find("DS")!=std::string::npos)
  {
    this->machine_type_=MachineType::Delivery;
    printf("deliv");
  }
  else
  {
    this->machine_type_=MachineType::Unknown;
    printf("unknowen");
  }
  printf("\n");
}

/** Called by the world update start event
 */
void Mps::OnUpdate(const common::UpdateInfo & /*_info*/)
{
}

/** on Gazebo reset
 */
void Mps::Reset()
{
}


/** Functions for recieving puck locations Messages
 * @param msg message
 */ 
void Mps::on_puck_msg(ConstPosePtr &msg)
{
  //printf("Got Msg from %s!!!", msg->name().c_str());

  //check if the puck is in the input area
  double dist = sqrt((msg->position().x() - input_x_) * (msg->position().x() - input_x_)
		     + (msg->position().y() - input_y_) * (msg->position().y() - input_y_)
		     + (msg->position().z() - BELT_HEIGHT) * (msg->position().z() - BELT_HEIGHT));
  if(dist < DETECT_TOLERANCE)
  {
    printf("Workpiece %s was inserted into %s.\n Telepoting it into output!\n", msg->name().c_str(), name_.c_str());
    //teleport puck to output
    model_->GetWorld()->GetEntity(msg->name())->SetWorldPose(math::Pose(output_x_, output_y_, BELT_HEIGHT, 0, 0, 0));
    //when this is a ring station spawn a ring ontop of the puck
    if(this->machine_type_==MachineType::Ring)
    {
      //write to the puck plugin
      std::string topic_string = std::string("~/") + msg->name() + std::string("/cmd");
      transport::PublisherPtr puck_cmd_pub = this->node_->Advertise<gazsim_msgs::WorkpieceCommand>(topic_string);
      if(!puck_cmd_pub->HasConnections())
      {
        printf("cannot connect to puck %s on topic %s\n",msg->name().c_str(),topic_string.c_str());
      }
      else
      {
        //TODO: dont'spawn a fixed color, get color from better source
        gazsim_msgs::WorkpieceCommand cmd;
        cmd.set_command(gazsim_msgs::Command::ADD_RING);
        cmd.set_color(gazsim_msgs::Color::BLUE);
        puck_cmd_pub->Publish(cmd);
      }
      puck_cmd_pub.reset();
    }
  }
  
}
