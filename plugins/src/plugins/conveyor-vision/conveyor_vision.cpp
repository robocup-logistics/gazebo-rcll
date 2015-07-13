/***************************************************************************
 *  conveyor_vision.cpp - Plugin for a conveyor_vision sensor on a model
 *
 *  Created: Sun Jul 12 16:02:29 2015
 *  Copyright  2015  Randolph Maaßen
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

#include "conveyor_vision.h"

using namespace gazebo;

// Register this plugin to make it available in the simulator
GZ_REGISTER_MODEL_PLUGIN(ConveyorVision)

ConveyorVision::ConveyorVision()
{
}

ConveyorVision::~ConveyorVision()
{
  printf("Destructing Conveyor Vision Plugin!\n");
}

/** on loading of the plugin
 * @param _parent Parent Model
 */
void ConveyorVision::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) 
{
  // Store the pointer to the model
  this->model_ = _parent;

  //get the model-name
  this->name_ = model_->GetName();
  printf("Loading Conveyor Vision Plugin of model %s\n", name_.c_str());

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&ConveyorVision::OnUpdate, this, _1));

  //Create the communication Node for communication with fawkes
  this->node_ = transport::NodePtr(new transport::Node());
  //the namespace is set to the model name!
  this->node_->Init(model_->GetWorld()->GetName()+"/"+name_);


  //create publisher
  this->conveyor_pub_ = this->node_->Advertise<llsf_msgs::ConveyorVisionResult>("~/RobotinoSim/ConveyorVisionResult/");

  //init last sent time
  last_sent_time_ = model_->GetWorld()->GetSimTime().Double();
  this->send_interval_ = 0.05;
}

/** Called by the world update start event
 */
void ConveyorVision::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  //Send gyro information to Fawkes
  double time = model_->GetWorld()->GetSimTime().Double();
  if(time - last_sent_time_ > send_interval_)
  {
    last_sent_time_ = time;
    send_conveyor_result();
  }
}

/** on Gazebo reset
 */
void ConveyorVision::Reset()
{
}

inline bool is_machine(gazebo::physics::ModelPtr model)
{
  std::string name = model->GetName();
  return ((name.find("BS") != std::string::npos) ||
          (name.find("CS") != std::string::npos) ||
          (name.find("DS") != std::string::npos) ||
          (name.find("RS") != std::string::npos));
}

inline double dist(gazebo::math::Pose p1, gazebo::math::Pose p2)
{
  gazebo::math::Pose dist = p1-p2;
  return sqrt(dist.pos.x*dist.pos.x + dist.pos.y*dist.pos.y + dist.pos.z*dist.pos.z);
}

void ConveyorVision::send_conveyor_result()
{
  if(!conveyor_pub_->HasConnections())
  {
    return;
  }
  gazebo::math::Pose bot_pose = model_->GetWorldPose();
  //for(gazebo::physics::ModelPtr model: model_->GetWorld()->GetModels())
  gazebo::physics::Model_V models = model_->GetWorld()->GetModels();
  for(gazebo::physics::Model_V::iterator it = models.begin(); it != models.end(); it++)
  {
    gazebo::physics::ModelPtr model = *it;
    if(is_machine(model) && dist(bot_pose, model->GetWorldPose()) < 1.5)
    {
      llsf_msgs::ConveyorVisionResult conv_msg;
      llsf_msgs::Pose3D *pose = new llsf_msgs::Pose3D();
      gazebo::math::Pose res = model->GetWorldPose() - bot_pose;
      res = res * bot_pose;
      pose->set_x(res.pos.x);
      pose->set_y(res.pos.y);
      pose->set_z(res.pos.z);
      pose->set_ori_x(res.rot.x);
      pose->set_ori_y(res.rot.y);
      pose->set_ori_z(res.rot.z);
      pose->set_ori_w(res.rot.w);
      conv_msg.set_allocated_positions(pose);
      //send
      conveyor_pub_->Publish(conv_msg);
      break;
    }
  }
}
