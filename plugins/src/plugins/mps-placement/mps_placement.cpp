/***************************************************************************
 *  mps_placement.cpp - Plugin to place the MPSs as specified by the refbox
 *
 *  Created: Thu Apr 16 12:44:00 2015
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
#include <time.h>
#include <cfloat>
#include <fnmatch.h>
#include <iostream>
#include <fstream>
#include <unistd.h>

#include <utils/misc/gazebo_api_wrappers.h>

#include "mps_placement.h"

using namespace gazebo;
using namespace gazebo_rcll;

///Constructor
MpsPlacementPlugin::MpsPlacementPlugin()
{
}
///Destructor
MpsPlacementPlugin::~MpsPlacementPlugin()
{
  printf("Destructing MpsPlacementPlugin Plugin!\n");
}

/** on loading of the plugin
 * @param _parent Parent Model
 */
void MpsPlacementPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/)
{
  // Store the pointer to the model
  world_ = _world;

  printf("Loading MpsPlacementPlugin WorldPlugin\n");

  //Create the communication Node for communication with fawkes
  this->node_ = transport::NodePtr(new transport::Node());
  //the namespace is set to the world name!
  this->node_->Init(world_->GZWRAP_NAME());

  //subscribe for refbox msgs
  machine_info_sub_ = node_->Subscribe(std::string(TOPIC_MACHINE_INFO), &MpsPlacementPlugin::on_machine_info_msg, this);
  game_state_sub_ = node_->Subscribe(std::string(TOPIC_GAME_STATE), &MpsPlacementPlugin::on_game_state_msg, this);

  machines_placed_ = false;
  is_game_started_ = false;
  random_seed_base_ = (int) time(NULL);

  factoryPub = node_->Advertise<msgs::Factory>("~/factory");
  modelPub = node_->Advertise<msgs::Model>("~/model");
}

/** on Gazebo reset
 */
void MpsPlacementPlugin::Reset()
{
  machines_placed_ = false;
}

/** Functions for recieving a machine info msg
 * @param msg message
 */
void MpsPlacementPlugin::on_machine_info_msg(ConstMachineInfoPtr &msg)
{

//        printf("GOT MACHINE INFO MESSAGE in MPS PLACEMENTsize %d\n", msg->machines_size());

        //printf("mplaced: %d gamestart%d\n", machines_placed_, is_game_started_);
  // don't set positions before simulation is initialized
  if(machines_placed_ || !is_game_started_ || world_->GZWRAP_SIM_TIME().Double() < WAIT_TIME_BEFORE_PLACEMENT){
    return;
  }

  //remove all existing mps (is broken at the moment, deleting models with plugins seems to be buggy in Gazebo, although it is working when deleting the model in the GUI)
  //remove_existing_mps();


  // go through all machines
  for(int i = 0; i < msg->machines_size(); i++){
    llsf_msgs::Machine machine_msg = msg->machines(i);
    std::string mps_name = machine_msg.name();

    //printf("Calculating Position for %s\n",mps_name.c_str());

    std::string zone = llsf_msgs::Zone_Name(msg->machines(i).zone());

    float offset_x = ZONE_WIDTH*0.5 ;
    float offset_y = ZONE_HEIGHT*0.5;
    float coord_x = 0;
    float coord_y = 0;

        switch (zone.at(0)) {
    case 'C':
        coord_x= ((int) zone.at(3) - '0') - offset_x;
        coord_y= ((int) zone.at(4) - '0') - offset_y;
            break;
    case 'M':
        //Magenta Team Zones are below Coordinate 0. -1 Mirrors Position on field
        coord_x= (((int) zone.at(3) - '0') - offset_x) * -1;
        coord_y= ((int) zone.at(4) - '0') - offset_y;
        break;
    default:
        printf("undefined zone color: %c\n",zone.at(0));
        return;
        break;
    }

    double ori = (M_PI *msg->machines(i).rotation())/180.0;
    ori -= M_PI/2; // substracting 90° to solve mismatch between refbox rotation and gazebo.

    //get machine type
    std::string mps_type;
    if(mps_name.find("BS") != std::string::npos){
      mps_type = "mps_base";
    } else if (mps_name.find("CS") != std::string::npos){
      mps_type = "mps_cap";
    } else if (mps_name.find("RS") != std::string::npos){
      mps_type = "mps_ring";
    } else if (mps_name.find("DS") != std::string::npos){
      mps_type = "mps_delivery";
    } else if (mps_name.find("SS") != std::string::npos){
      mps_type = "mps_storage";
    } else {
      printf("Unknown mps-type: %s", mps_name.c_str());
      return;
    }

    msgs::Factory spawn_mps_msg;
    //get sdf, replaced name and set it to the factory message
    std::string sdf_path = getenv("GAZEBO_RCLL");
    sdf_path += "/models/" + mps_type + "/model.sdf";
    std::ifstream raw_sdf_file(sdf_path.c_str());
    std::string new_sdf;
    if (raw_sdf_file.is_open()){
      std::string raw_sdf((std::istreambuf_iterator<char>(raw_sdf_file)),
                    std::istreambuf_iterator<char>());
      std::size_t name_pos = raw_sdf.find(mps_type);
      if(name_pos ==  std::string::npos){
          printf("SDF file %s has no model named %s\n",sdf_path.c_str(), mps_type.c_str());
        return;
      }
      new_sdf = raw_sdf.erase(name_pos, mps_type.length()).insert(name_pos, mps_name);
    }
    else{
      printf("Cant find mps sdf file:%s\n", sdf_path.c_str());
      return;
    }
    spawn_mps_msg.set_sdf(new_sdf.c_str());
    spawn_mps_msg.set_clone_model_name(mps_name.c_str());
#if GAZEBO_MAJOR_VERSION > 5
    msgs::Set(spawn_mps_msg.mutable_pose(),
              ignition::math::Pose3d(coord_x, coord_y, 0, 0, 0, ori));
#else
    msgs::Set(spawn_mps_msg.mutable_pose(),
              math::Pose(coord_x, coord_y, 0, 0, 0, ori));
#endif

    printf("Place MPS %s in Zone: name: %s Pos: (%f,%f) ori: %d\n",mps_name.c_str(), zone.c_str(), coord_x,coord_y, msg->machines(i).rotation());
    factoryPub->Publish(spawn_mps_msg);
  }
  printf("MpsPlacementPlugin: All machines placed\n");
  machines_placed_ = true;
}

/** Functions for recieving a game state msg
 * We want to knwo if the game is started because the refbox assigns
 *  the zones to the machines not in the PRE_GAME phase
 * @param msg message
 */
void MpsPlacementPlugin::on_game_state_msg(ConstGameStatePtr &msg)
{
  if(!is_game_started_ && msg->phase() != llsf_msgs::GameState::PRE_GAME){
    printf("MpsPlacementPlugin: Game started\n");
    is_game_started_ = true;
  }
  // Not used because removing mps does not work at the moment
  // if(is_game_started_ && msg->phase() == llsf_msgs::GameState::PRE_GAME){
  //   printf("MpsPlacementPlugin: Game stopped\n");
  //   is_game_started_ = false;
  //   machines_placed_ = false;
  // }
}

void MpsPlacementPlugin::spawn_mps(const gzwrap::Pose3d &spawn_pose, std::string model_name)
{
  printf("spawning mps %s\n", model_name.c_str());
  msgs::Factory spawn_msg;
  spawn_msg.set_sdf_filename(model_name.c_str());
#if GAZEBO_MAJOR_VERSION > 5 && GAZEBO_MAJOR_VERSION < 8
  msgs::Set(spawn_msg.mutable_pose(), spawn_pose.Ign());
#else
  msgs::Set(spawn_msg.mutable_pose(), spawn_pose);
#endif
  factoryPub->Publish(spawn_msg);
}

void MpsPlacementPlugin::remove_existing_mps()
{
  printf("Removing existing MPS\n");
  unsigned int modelCount = world_->GZWRAP_MODEL_COUNT();
  for(unsigned int i = 0 ; i < modelCount; i++){
    physics::ModelPtr mps = world_->GZWRAP_MODEL_BY_INDEX(i);
    if (fnmatch("puck*",mps->GetName().c_str(),FNM_CASEFOLD) == 0
        || fnmatch("puck*",mps->GetName().c_str(),FNM_CASEFOLD) == 0){
      // printf("Remove Plugin\n");
      // world_->RemovePlugin(mps->GetName() + "::MpsLoader");
      printf("Deleting %s\n", mps->GetName().c_str());
      // mps->Fini();
      msgs::Model del_msg;
      del_msg.set_name(mps->GetName().c_str());
      del_msg.set_deleted(false);
      modelPub->Publish(del_msg);
      printf("Deleted \n");
    }
  }
}
