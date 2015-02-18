/***************************************************************************
 *  robotino.cpp - Main Plugin file for controlling the robotino model
 *
 *  Created: Mon Jul 29 17:33:31 2013
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
#include <math.h>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/RaySensor.hh>

#include "robotino.h"
#include "simDevice.h"
#include "config.h"
#include "messageDisplay.h"
#include "machineVision.h"
#include "puck_detection.h"
#include "infraredPuckSensor.h"
#include "gripper_laser_sensor.h"
#include "puck_holder.h"

using namespace gazebo;

// Register this plugin to make it available in the simulator
GZ_REGISTER_MODEL_PLUGIN(Robotino)


Robotino::Robotino()
{
}

Robotino::~Robotino()
{
  printf("Destructing Robotino Plugin!\n");
  //Destruct all simulated devices
  for (std::list<SimDevice*>::iterator it = devices_list_.begin(); it != devices_list_.end(); it++)
  {
    delete *it;
  }
}

/** Loading the Robotino plugin
 * @param _parent model the plugin is attached to
 */
void Robotino::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) 
{
  // Store the pointer to the model
  this->model_ = _parent;

  //get the model-name
  this->name_ = model_->GetName();
  printf("Loading Robotino Plugin of model %s\n", name_.c_str());

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&Robotino::OnUpdate, this, _1));

  //Create the communication Node for communication with fawkes
  this->node_ = transport::NodePtr(new transport::Node());
  
  //Create the communication node for spawning the number label
  this->visual_node_ = transport::NodePtr(new transport::Node());
  visual_node_->Init(model_->GetWorld()->GetName().c_str());
  label_last_sent_ = 0.0;

  if(name_ == "robotino")
  {
    printf("UNNAMED ROBOTINO LOADED! INITIALIZING GAZEBO NODE COMMUNUNICATION WITH DEFAULT CHANNEL \"\"!");
    this->node_->Init();
  }
  else
  {
    this->node_->Init(model_->GetWorld()->GetName()+"/"+name_);
  } 
  
  //creating simulated devices
  devices_list_.push_back((SimDevice*) new MessageDisplay(model_, node_));
  devices_list_.push_back((SimDevice*) new MachineVision(model_, node_));
  devices_list_.push_back((SimDevice*) new PuckDetection(model_, node_));
  std::string infrared_name =  model_->GetWorld()->GetName() + "::" + name_ + "::infrared_sensor::link::infrared_puck_sensor";
  devices_list_.push_back((SimDevice*) new InfraredPuckSensor(model_, node_, sensors::get_sensor(infrared_name.c_str())));
  std::string gripper_laser_left_name =  model_->GetWorld()->GetName() + "::" + name_ + "::body::gripper_laser_left";
  std::string gripper_laser_right_name =  model_->GetWorld()->GetName() + "::" + name_ + "::body::gripper_laser_right";
  devices_list_.push_back((SimDevice*) new GripperLaserSensor(model_, node_, sensors::get_sensor(gripper_laser_left_name.c_str()), LEFT));
  devices_list_.push_back((SimDevice*) new GripperLaserSensor(model_, node_, sensors::get_sensor(gripper_laser_right_name.c_str()), RIGHT));

  if(ATTACH_PUCK_TO_GRIPPER_WHEN_TURNING)
  {
    devices_list_.push_back((SimDevice*) new PuckHolder(model_, node_));
  }

  //initialize and publish messages of devices (before subscribing to avoid deadlocks)
  for (std::list<SimDevice*>::iterator it = devices_list_.begin(); it != devices_list_.end(); it++)
  {
    (*it)->init();
    (*it)->create_publishers();
  }
  visual_pub_ = visual_node_->Advertise<msgs::Visual>("~/visual", 1);

  //suscribe messages of devices
  for (std::list<SimDevice*>::iterator it = devices_list_.begin(); it != devices_list_.end(); it++)
  {
    (*it)->create_subscribers();
  }

  printf("Robotino-Plugin sucessfully loaded \n");
}


/** Called by the world update start event
 */ 
void Robotino::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  //update devices
  for (std::list<SimDevice*>::iterator it = devices_list_.begin(); it != devices_list_.end(); it++)
  {
    (*it)->update();
  }

  // double time = model_->GetWorld()->GetSimTime().Double();  
  // if(time - label_last_sent_ > 5.0)
  // {
  //   spawn_label();
  //   label_last_sent_ = time;
  // }
}

/** What to do on resetting of the plugin
 */
void Robotino::Reset()
{
}

void Robotino::spawn_label()
{
  double time = model_->GetWorld()->GetSimTime().Double();
  //wait until the world is completly loaded, otherwise the label will spawn at (0,0)
  if(time < 12)
  {
    return;
  }
  
  //create message
  msgs::Visual msg;

  //set parameters
  msg.set_name((name_ + "::label").c_str());
  std::string parent = model_->GetName() + "::body";
  msg.set_parent_name(parent.c_str());
#if GAZEBO_MAJOR_VERSION >= 2  
  msg.set_parent_id(model_->GetId());
#endif
  msgs::Geometry *geomMsg = msg.mutable_geometry();
  geomMsg->set_type(msgs::Geometry::PLANE);
  msgs::Set(geomMsg->mutable_plane()->mutable_normal(), math::Vector3(0.0, 0.0, 1.0));
  msgs::Set(geomMsg->mutable_plane()->mutable_size(), math::Vector2d(0.2, 0.2));
  msg.set_transparency(0.5);  
  msg.set_cast_shadows(false);
  msgs::Set(msg.mutable_pose(), math::Pose(0.00, 0.0, 0.4, 0, 0, 0));
  
  //find right number and set texture
  msgs::Material::Script* script = msg.mutable_material()->mutable_script();
  script->add_uri("model://label/materials/scripts");
  script->add_uri("model://label/materials/textures");
  std::string label_name = "label/" + name_;
  script->set_name(label_name.c_str());

  //Publish
  visual_pub_->Publish(msg);
}
