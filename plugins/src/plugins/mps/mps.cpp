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
//GZ_REGISTER_MODEL_PLUGIN(Mps)

///Constructor
Mps::Mps(physics::ModelPtr _parent, sdf::ElementPtr)
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

  spawned_tags_last_ = model_->GetWorld()->GetSimTime().Double();

  //subscribe for puck locations
  for(int i = 0; i < NUMBER_PUCKS; i++)
  {
    this->puck_subs_[i] = this->node_->Subscribe(std::string("~/puck_") + std::to_string(i) + "/gazsim/gps/" , &Mps::on_puck_msg, this);
  }

  //Create publisher to spawn tags
  visPub_ = this->node_->Advertise<msgs::Visual>("~/visual", /*number of lights*/ 3*12);

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
}
///Destructor
Mps::~Mps()
{
  printf("Destructing Mps Plugin!\n");
}

/** Called by the world update start event
 */
void Mps::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  if(model_->GetWorld()->GetSimTime().Double() - spawned_tags_last_ > TAG_SPAWN_TIME)
  {
    //Spawn tags (in Init is to early because it would be spawned at origin)
    spawnTag("tag_input", name_ + "I" , 0, -0.176, 0);
    spawnTag("tag_output", name_ + "O" , 0, 0.176, 3.14);
    spawned_tags_last_ = model_->GetWorld()->GetSimTime().Double();
  }
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

}

/**
 * Spawn tag on machine sides
 */
void Mps::spawnTag(std::string visual_name, std::string tag_name, float x, float y, float ori)
{
  //create message to return
  msgs::Visual msg;

  msgs::Geometry *geomMsg = msg.mutable_geometry();
  geomMsg->set_type(msgs::Geometry::PLANE);
  
  msgs::Set(geomMsg->mutable_plane()->mutable_normal(), math::Vector3(0, 0, 1));
  msgs::Set(geomMsg->mutable_plane()->mutable_size(), math::Vector2d(TAG_SIZE, TAG_SIZE));
  msg.set_cast_shadows(false);

  //construct full path to link that should contain the tag
  std::string parent_link = name_ + "::mps_tags::link";
  msg.set_parent_name(parent_link.c_str());

  msg.set_name((parent_link + "::" + visual_name).c_str());
  msgs::Set(msg.mutable_pose(), math::Pose(x, y, TAG_HEIGHT, 1.57, 0, ori));

  //set right texture
  msg.mutable_material()->mutable_script()->set_name(std::string("tag/") + tag_name);

  std::string *uri1 = msg.mutable_material()->mutable_script()->add_uri();
  *uri1 = "model://tags/materials/scripts";
  std::string *uri2 = msg.mutable_material()->mutable_script()->add_uri();
  *uri2 = "model://tags/materials/textures";
  visPub_->Publish(msg);
}
