/***************************************************************************
 *  tag.cpp - Plugin to spawn the right tag pattern and publish the pose
 *            The name of the spawned model has to be something like
 *            'prefix/tag_01/suffix' to display the right tag pattern
 *
 *  Created: Fri Oct 16 18:00:08 2015
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
#include <iostream>
#include <fstream>
#include <fnmatch.h>
#include <stdlib.h>

#include "tag.h"

using namespace gazebo;

// Register this plugin to make it available in the simulator
GZ_REGISTER_MODEL_PLUGIN(Tag)

///Constructor
Tag::Tag()
{
}

/** on loading of the plugin
 * @param _parent Parent Model
 */
void Tag::Load(physics::ModelPtr _parent, sdf::ElementPtr)
{
  // Store the pointer to the model
  this->model_ = _parent;

  //get the model-name
  this->name_ = model_->GetName();
  printf("Loading Tag Plugin of model %s\n", name_.c_str());

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&Tag::OnUpdate, this, _1));

  //Create the communication Node for communication with fawkes
  this->node_ = transport::NodePtr(new transport::Node());
  //the namespace is set to the world name!
  this->node_->Init(model_->GetWorld()->GetName());

  created_time_ = model_->GetWorld()->GetSimTime().Double();
  spawned_tags_last_ = model_->GetWorld()->GetSimTime().Double();

  //Create publisher to spawn tags
  visPub_ = this->node_->Advertise<msgs::Visual>("~/visual");

  world_ = model_->GetWorld();
}

///Destructor
Tag::~Tag()
{
  printf("Destructing Tag Plugin for %s!\n",this->name_.c_str());
}

/** Called by the world update start event
 */
void Tag::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  if(model_->GetWorld()->GetSimTime().Double() - spawned_tags_last_ > TAG_SPAWN_TIME)
  {
    //Spawn tags (in Init is to early because it would be spawned at origin)

    //create message
    msgs::Visual msg;

    msgs::Geometry *geomMsg = msg.mutable_geometry();
    geomMsg->set_type(msgs::Geometry::PLANE);
  
#if GAZEBO_MAJOR_VERSION > 5
    msgs::Set(geomMsg->mutable_plane()->mutable_normal(), ignition::math::Vector3d(0, 0, 1));
    msgs::Set(geomMsg->mutable_plane()->mutable_size(), ignition::math::Vector2d(TAG_SIZE, TAG_SIZE));
#else
    msgs::Set(geomMsg->mutable_plane()->mutable_normal(), math::Vector3(0, 0, 1));
    msgs::Set(geomMsg->mutable_plane()->mutable_size(), math::Vector2d(TAG_SIZE, TAG_SIZE));
#endif
    msg.set_cast_shadows(false);

    //construct full path to link that should contain the tag
    std::string parent_link = name_ + "::link";
    msg.set_parent_name(parent_link.c_str());

    msg.set_name((parent_link + "::pattern").c_str());
#if GAZEBO_MAJOR_VERSION > 5
    msgs::Set(msg.mutable_pose(), ignition::math::Pose3d(0, 0, 0.001, 0, 0, 0));
#else
    msgs::Set(msg.mutable_pose(), math::Pose(0, 0, 0.001, 0, 0, 0));
#endif

    //get tag-id from name
    if(name_.find("tag_") == std::string::npos){
      printf("Tag: can not create tag pattern because the model name of %s has not the format 'prefix/tag_01/suffix'!!\n", name_.c_str());
      return;
    }
    std::string tag_id = name_.substr(name_.find("tag_"));
    if(tag_id.find("/") != std::string::npos){
      tag_id = tag_id.substr(0, tag_id.find("/"));
    }
    // printf("Tag: creating tag pattern %s\n", tag_id.c_str());
    //set right texture (here the model name has to be tag_id)
    msg.mutable_material()->mutable_script()->set_name(std::string("tag/") + tag_id);

    std::string *uri1 = msg.mutable_material()->mutable_script()->add_uri();
    *uri1 = "model://tag/materials/scripts";
    std::string *uri2 = msg.mutable_material()->mutable_script()->add_uri();
    *uri2 = "model://tag/materials/textures";
    visPub_->Publish(msg);

    spawned_tags_last_ = model_->GetWorld()->GetSimTime().Double();
  }
}

/** on Gazebo reset
 */
void Tag::Reset()
{
}
