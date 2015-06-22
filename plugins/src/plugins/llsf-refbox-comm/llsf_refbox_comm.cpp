/***************************************************************************
 *  llsf_refbox_comm.cpp - World plugin for the refbox connection in the llsf
 *
 *  Created: Fri Mar 06 16:10:09 2015
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

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <string.h>
#include <cstdlib>
#include <protobuf_comm/client.h>
#include <protobuf_comm/message_register.h>

#include "llsf_refbox_comm.h"

using namespace gazebo;

LlsfRefboxCommPlugin::LlsfRefboxCommPlugin() : WorldPlugin() 
{
  //Init the communication Node
  this->node_ = transport::NodePtr(new transport::Node());
  this->node_->Init("LLSF");

  //resolve path to proto dirs by using the environmental variable $GAZEBO_RCLL
  const char * folder_path = ::getenv("GAZEBO_RCLL");
  if ( folder_path == 0 ) {
    printf("\n\n\nCan not find $GAZEBO_RCLL. Please set it in your .bashrc to the path to the gazebo-rcll folder.\n\n\n");
    return;
  }
  else {
    proto_dirs_ = {std::string(folder_path) + PROTO_DIR};
  }
}

LlsfRefboxCommPlugin::~LlsfRefboxCommPlugin() 
{
}

/** Initialization while loading the plugin
 * @param _world World where the plugi was loaded
 * @param _sdf Pointer to the sdf model definition
 */
void LlsfRefboxCommPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  world_ = _world;
  
  //create publisher and subscriber for connection with gazebo node
  machine_info_pub_ = node_->Advertise<llsf_msgs::MachineInfo>(TOPIC_MACHINE_INFO);
  game_state_pub_ = node_->Advertise<llsf_msgs::GameState>(TOPIC_GAME_STATE);
  // puck_info_pub_ = node_->Advertise<llsf_msgs::PuckInfo>(config->get_string("/gazsim/topics/puck-info"));
  // place_puck_under_machine_sub_ = node_->Subscribe(config->get_string("/gazsim/topics/place-puck-under-machine"), &LlsfRefboxCommPlugin::on_puck_place_msg, this);
  // remove_puck_under_machine_sub_ = node_->Subscribe(config->get_string("/gazsim/topics/remove-puck-under-machine"), &LlsfRefboxCommPlugin::on_puck_remove_msg, this);
  time_sync_sub_ = node_->Subscribe(TOPIC_TIME, &LlsfRefboxCommPlugin::on_time_sync_msg, this);
  set_game_state_sub_ = node_->Subscribe(TOPIC_SET_GAME_STATE, &LlsfRefboxCommPlugin::on_set_game_state_msg, this);
  set_game_phase_sub_ = node_->Subscribe(TOPIC_SET_GAME_PHASE, &LlsfRefboxCommPlugin::on_set_game_phase_msg, this);
  set_team_name_sub_ = node_->Subscribe(TOPIC_SET_TEAM_NAME, &LlsfRefboxCommPlugin::on_set_team_name_msg, this);  
  set_machine_state_sub_ = node_->Subscribe(TOPIC_SET_MACHINE_STATE, &LlsfRefboxCommPlugin::on_set_machine_state_msg, this);  

  //connect update function
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&LlsfRefboxCommPlugin::Update, this));
  printf("LLSF-refbox-connection-Plugin loaded!\n");

  connected_ = false;
  connect_tries_ = 0;
  printf("Trying to connect to refbox\n");
  //prepare client
  create_client();
  client_->async_connect(REFBOX_HOST, REFBOX_PORT);

  last_connect_try_ = world_->GetSimTime().Double();
}

void LlsfRefboxCommPlugin::Update()
{ 
  if(!connected_ && connect_tries_ < RECONNECT_ATTEMPTS)
  {
    //if not connected, try to reconnect every x seconds
    double time = world_->GetSimTime().Double();
    if((time - last_connect_try_) > RECONNECT_INTERVAL)
    {
      connect_tries_++;
      printf("Trying to connect to refbox\n");
      last_connect_try_ = time;
      create_client();
      client_->async_connect(REFBOX_HOST, REFBOX_PORT);
    }
  }
}
/** Handler for successful connection to the client
 */
void
LlsfRefboxCommPlugin::client_connected()
{
  printf("LLSF-refbox-comm: Connected to Refbox\n");
  connected_ = true;
}


/** Handler for loss of connection to client
 * @param error boost error code
 */
void
LlsfRefboxCommPlugin::client_disconnected(const boost::system::error_code &error)
{
  printf("LLSF-refbox-comm: Disconnected\n");
  connected_ = false;
  
  if(connect_tries_ == RECONNECT_ATTEMPTS){
    printf("LLSF-refbox-comm: Refbox-connect failed %d times. Stop trying to connect.\n", RECONNECT_ATTEMPTS);
  }
}

/** Handler for incoming msg from client
 * @param comp_id component id of protobuf msg
 * @param msg_type type of protobuf msg
 * @param msg pointer to protobuf msg
 */
void
LlsfRefboxCommPlugin::client_msg(uint16_t comp_id, uint16_t msg_type,
			     std::shared_ptr<google::protobuf::Message> msg)
{
  //printf("LLSF-refbox-comm: Got Message from refbox: %s\n", msg->GetTypeName().c_str());
  
  //Filter wanted messages
  if(msg->GetTypeName() == "llsf_msgs.MachineInfo")
  {
    machine_info_pub_->Publish(*msg);
    return;
  }
  
  if(msg->GetTypeName() == "llsf_msgs.GameState")
  {
    game_state_pub_->Publish(*msg);
    return;
  }
  
  // if(msg->GetTypeName() == "llsf_msgs.PuckInfo")
  // {
  //   //logger->log_info(name(), "Sending PuckInfo to gazebo");
  //   puck_info_pub_->Publish(*msg);
  //   return;
  // }
}

void LlsfRefboxCommPlugin::create_client()
{
  //create message register with all messages to listen for
  message_register_ = new protobuf_comm::MessageRegister(proto_dirs_);

  //create client and register handlers
  client_ = new protobuf_comm::ProtobufStreamClient(message_register_);
  client_->signal_connected().connect(
    boost::bind(&LlsfRefboxCommPlugin::client_connected, this));
  client_->signal_disconnected().connect(
    boost::bind(&LlsfRefboxCommPlugin::client_disconnected,
  		this, boost::asio::placeholders::error));
  client_->signal_received().connect(
    boost::bind(&LlsfRefboxCommPlugin::client_msg, this, _1, _2, _3));
}

// void LlsfRefboxCommPlugin::on_puck_place_msg(ConstPlacePuckUnderMachinePtr &msg)
// {
// }

// void LlsfRefboxCommPlugin::on_puck_remove_msg(ConstRemovePuckFromMachinePtr &msg)
// {
// }

void LlsfRefboxCommPlugin::on_time_sync_msg(ConstSimTimePtr &msg)
{
  // printf("LLSF-refbox-comm: Sending Simulation Time\n");
  
  //provide time source with newest message
  if(!connected_)
  {
    return;
  }
  //fill msg for refbox with info from gazsim_msg
  llsf_msgs::SimTimeSync to_rb;
  llsf_msgs::Time* time = to_rb.mutable_sim_time();
  time->set_sec(msg->sim_time_sec());
  time->set_nsec(msg->sim_time_nsec());
  to_rb.set_real_time_factor(msg->real_time_factor());
  to_rb.set_paused(msg->paused());

  //send it and make refbox able to handle the msg
  client_->send(to_rb);
}

void LlsfRefboxCommPlugin::on_set_game_state_msg(ConstSetGameStatePtr &msg)
{
  if(!connected_)
  {
    return;
  }
  llsf_msgs::SetGameState to_rb = *msg;
  client_->send(to_rb);
}

void LlsfRefboxCommPlugin::on_set_game_phase_msg(ConstSetGamePhasePtr &msg)
{
  if(!connected_)
  {
    return;
  }
  llsf_msgs::SetGamePhase to_rb = *msg;
  client_->send(to_rb);
}

void LlsfRefboxCommPlugin::on_set_team_name_msg(ConstSetTeamNamePtr &msg)
{
  if(!connected_)
  {
    return;
  }
  llsf_msgs::SetTeamName to_rb = *msg;
  client_->send(to_rb);
}

void LlsfRefboxCommPlugin::on_set_machine_state_msg(ConstSetMachineStatePtr &msg)
{
  if(!connected_)
  {
    return;
  }
  llsf_msgs::SetMachineState to_rb = *msg;
  client_->send(to_rb);
}
