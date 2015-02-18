/***************************************************************************
 *  puck_detection.cpp - provides puck-positions
 *
 *  Created: Thu Aug 29 10:05:15 2013
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
#include "puck_detection.h"
#include <llsf_msgs/PuckDetectionResult.pb.h>
#include "../llsf/data_table.h"
#include "config.h"

using namespace gazebo;

/** Constructor
 * @param model Model to the gazebo model of the plugin
 * @param node Transport node to publish and subscribe messages on
 */
PuckDetection::PuckDetection(physics::ModelPtr model, transport::NodePtr node)
 : SimDevice(model, node)
{
}
PuckDetection::~PuckDetection()
{
}

void PuckDetection::init()
{
  printf("Initialize PuckDetection \n");
  table_ = LlsfDataTable::get_table();
}

void PuckDetection::create_publishers()
{
  this->puck_position_pub_ = this->node->Advertise<llsf_msgs::PuckDetectionResult>("~/RobotinoSim/PuckDetectionResult/");
}

void PuckDetection::create_subscribers()
{
  //no subscribers
}

void PuckDetection::update()
{
  //Send position information to Fawkes
  double time = model->GetWorld()->GetSimTime().Double();
  if(time - last_sent_time_ > (1.0 / PUCK_DETECTION_SEND_FREQUENCY))
  {
    last_sent_time_ = time;
    send_puck_positions();
  }
}

void PuckDetection::send_puck_positions()
{
  //send ground truth puck positions to fawkes
  //fawkes chooses which of them are too far away to detect
  
  //build Protobuf Message
  llsf_msgs::PuckDetectionResult msg;
  for(int i = 0; i < NUMBER_PUCKS; i++)
  {
    llsf_msgs::Pose2D *pose = msg.add_positions();
    //get puck from table
    Puck puck = table_->get_puck(i);

    //set x, y (relative to robotino position)
    double robot_x = this->model->GetWorldPose().pos.x;
    double robot_y = this->model->GetWorldPose().pos.y;
    double robot_ori = this->model->GetWorldPose().rot.GetAsEuler().z;
    double rel_x = puck.x - robot_x;
    double rel_y = puck.y - robot_y;
    //apply transformation regarding robot orientation
    double omni_x = rel_x * cos(-robot_ori) - rel_y * sin(-robot_ori);
    double omni_y = rel_x *sin(-robot_ori) + rel_y * cos(-robot_ori);
    pose->set_x(omni_x);
    pose->set_y(omni_y);

    //set fake timestamp and ori
    pose->mutable_timestamp()->set_sec(0);
    pose->mutable_timestamp()->set_nsec(0);
    pose->set_ori(0);
  }
  //send it
  puck_position_pub_->Publish(msg);
}
