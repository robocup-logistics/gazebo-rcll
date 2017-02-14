/***************************************************************************
 *  light-signal-detection.cpp - provides ground truth light signal detection of the nearest achine in front of the robotino
 *  Created: Mon Mar 30 16:28:38 2015
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

#include "light-signal-detection.h"
#include <llsf_msgs/LightSignals.pb.h>
#include <llsf_msgs/MachineInfo.pb.h>
#include <gazsim_msgs/LightSignalDetection.pb.h>

using namespace gazebo;

// Register this plugin to make it available in the simulator
GZ_REGISTER_MODEL_PLUGIN(LightSignalDetection)

///Constructor
LightSignalDetection::LightSignalDetection()
{
}
///Destructor
LightSignalDetection::~LightSignalDetection()
{
  printf("Destructing LightSignalDetection Plugin!\n");
}

/** on loading of the plugin
 * @param _parent Parent Model
 */
void LightSignalDetection::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) 
{
  // Store the pointer to the model
  this->model_ = _parent;

  //get the model-name
  this->name_ = model_->GetName();
  printf("Loading LightSignalDetection Plugin of model %s\n", name_.c_str());

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&LightSignalDetection::OnUpdate, this, _1));

  //Create the communication Node for communication with fawkes
  this->node_ = transport::NodePtr(new transport::Node());
  //the namespace is set to the model name!
  this->node_->Init(model_->GetWorld()->GZWRAP_NAME()+"/"+name_);

  //Create the communication Node in gazbeo
  this->world_node_ = transport::NodePtr(new transport::Node());
  //the namespace is set to the world name!
  this->world_node_->Init(model_->GetWorld()->GZWRAP_NAME());


  //init last sent time
  last_sent_time_ = model_->GetWorld()->GZWRAP_SIM_TIME().Double();

  //create publisher
  this->light_signal_pub_ = this->node_->Advertise<gazsim_msgs::LightSignalDetection>("~/gazsim/light-signal/");

  //subscribe for light status msgs
  light_msg_sub_ = world_node_->Subscribe(std::string(TOPIC_MACHINE_INFO), &LightSignalDetection::on_light_msg, this);
  
  robot_pose_ = model_->GZWRAP_WORLD_POSE();

  //initial values:
  visible_ = false;
  visibility_history_ = -1;
  visible_since_ = 0;
  state_red_ = state_green_ = state_yellow_ = llsf_msgs::OFF;
}


/** Called by the world update start event
 */
void LightSignalDetection::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  //Safe robot position to know if a light signal is in front of the robot
  robot_pose_ = model_->GZWRAP_WORLD_POSE();
  //send message to robot control software periodically:
  double time = model_->GetWorld()->GZWRAP_SIM_TIME().Double();
  if(time - last_sent_time_ > SEND_INTERVAL && visible_)
  {
    last_sent_time_ = time;
    //set visibility history
    visibility_history_ = (time - visible_since_) * VISIBILITY_HISTORY_INCREASE_PER_SECOND;
    send_light_detection();
  }
}

/** on Gazebo reset
 */
void LightSignalDetection::Reset()
{
}

/** Sending position to Fawkes
 * 
 */
void LightSignalDetection::send_light_detection()
{
  if(light_signal_pub_->HasConnections())
  {
    gazsim_msgs::LightSignalDetection msg;
    msg.set_visible(visible_);
    msg.set_visibility_history(visibility_history_);
    gazsim_msgs::LightSignalDetection::LightSpec* red = msg.add_lights();
    gazsim_msgs::LightSignalDetection::LightSpec* yellow = msg.add_lights();
    gazsim_msgs::LightSignalDetection::LightSpec* green = msg.add_lights();
    red->set_color(gazsim_msgs::LightSignalDetection::RED);
    yellow->set_color(gazsim_msgs::LightSignalDetection::YELLOW);
    green->set_color(gazsim_msgs::LightSignalDetection::GREEN);
    red->set_state((gazsim_msgs::LightSignalDetection::LightState)state_red_);
    yellow->set_state((gazsim_msgs::LightSignalDetection::LightState)state_yellow_);
    green->set_state((gazsim_msgs::LightSignalDetection::LightState)state_green_);

    light_signal_pub_->Publish(msg);
  }
}

/** Functions for recieving a light signal status msg
 * @param msg message
 */ 
void LightSignalDetection::on_light_msg(ConstMachineInfoPtr &msg)
{
  // printf("LightSignalDetection: Got Light Msg!\n");

  //Calculate Robot detetion center
  double look_pos_x = robot_pose_.GZWRAP_POS_X
    + cos(robot_pose_.GZWRAP_ROT_YAW) * SEARCH_AREA_REL_X - sin(robot_pose_.GZWRAP_ROT_YAW) * SEARCH_AREA_REL_Y;
  double look_pos_y = robot_pose_.GZWRAP_POS_Y
    + sin(robot_pose_.GZWRAP_ROT_YAW) * SEARCH_AREA_REL_X + cos(robot_pose_.GZWRAP_ROT_YAW) * SEARCH_AREA_REL_Y;

  
  // find mearest machine in front of the robot
  int nearest_index = -1;
  float min_dist = 1000000;
  for(int i = 0; i < msg->machines_size(); i++){
    llsf_msgs::Machine machine = msg->machines(i);
    std::string machine_name = machine.name();
    std::string light_link_name = machine_name + "::light_signals::link";
    physics::EntityPtr light_entity = model_->GetWorld()->GZWRAP_ENTITY_BY_NAME(light_link_name.c_str());
    if(light_entity == NULL){
	    //printf("Light-Signal-Detection can't find machine with name %s!\n", machine_name.c_str());
      return;
    }
    gzwrap::Pose3d light_pose = light_entity->GZWRAP_WORLD_POSE();
    float dist = light_pose.GZWRAP_POS.Distance(look_pos_x, look_pos_y, light_pose.GZWRAP_POS_Z);
    if(dist < min_dist){
      min_dist = dist;
      nearest_index = i;
    }
  }

  // get machine message of nearest machine
  if(min_dist < RADIUS_DETECTION_AREA){
    //check if the signal changed
    llsf_msgs::LightState old_red = state_red_;
    llsf_msgs::LightState old_yellow = state_yellow_;
    llsf_msgs::LightState old_green = state_green_;
    save_light_signal(msg->machines(nearest_index));
    if(!visible_
       || old_red != state_red_
       || old_yellow != state_yellow_
       || old_green != state_green_)
    {
      //something changed
      visible_ = true;
      visibility_history_ = 0;
      visible_since_ = model_->GetWorld()->GZWRAP_SIM_TIME().Double();
    }
    //light detection is sent periodically in the update loop
  }
  else{
    visible_ = false;
    visibility_history_ = -1;
    send_light_detection();
  }
}

void LightSignalDetection::save_light_signal(llsf_msgs::Machine machine)
{
  //go through all light specs
  //set default values
  state_red_ = llsf_msgs::OFF;
  state_yellow_ = llsf_msgs::OFF;
  state_green_ = llsf_msgs::OFF;
  for(int i = 0; i < machine.lights_size(); i++){
    llsf_msgs::LightSpec light_msg = machine.lights(i);
    switch(light_msg.color())
    {
    case llsf_msgs::RED: state_red_ = light_msg.state(); break;
    case llsf_msgs::YELLOW: state_yellow_ = light_msg.state(); break;
    case llsf_msgs::GREEN: state_green_ = light_msg.state(); break;
    }
  }
}
