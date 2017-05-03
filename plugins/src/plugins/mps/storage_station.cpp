/***************************************************************************
 *  base_station.cpp - controls a basesation mps
 *
 *  Generated: Wed Apr 22 13:25:39 2015
 *  Copyright  2015  Randolph Maaßen
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include "storage_station.h"

using namespace gazebo;

StorageStation::StorageStation(physics::ModelPtr _parent, sdf::ElementPtr _sdf) :
  Mps(_parent,_sdf)
{
    storage_ = new std::string[SLOT_Z_COUNT * SLOT_Y_COUNT * SLOT_X_COUNT];

    //EMPTY all storage slots
    for (int i=0;i<SLOT_Z_COUNT * SLOT_Y_COUNT * SLOT_X_COUNT; i++){
        storage_[i] ="";
    }

    storage_cnt = 0;

    shelf_pos_x = SHELF_POS_X;
    shelf_pos_z = SHELF_POS_Z;

    if (name_.find("M-") != std::string::npos){

        shelf_pos_y = 0.0;
        printf("Strage_Station: Generating Puck Storage Magenta\n");
    }else{
        shelf_pos_y = 10.0;
        printf("Strage_Station: Generating Puck Storage Cyan\n");
    }

    for (int z=0; z<SLOT_Z_COUNT;z++){
        for (int y=0; y<SLOT_Y_COUNT; y++)
            for (int x=0; x<SLOT_X_COUNT; x++){
            spawn_puck(get_slot_World_position(x,y,z),gazsim_msgs::Color::RED);
            }
    }
}

StorageStation::~StorageStation(){

    delete storage_;
}


void StorageStation::on_puck_msg(ConstPosePtr &msg)
{

    //printf("%s: PUCK_MSG: %s\n",name_.c_str(),msg->name().c_str());

    /*
  if(msg->name() == have_puck_ &&
     !puck_in_input(msg) &&
     !puck_in_output(msg))
  {
    have_puck_ = "";
    set_state(State::RETRIEVED);
  }
  */
}

void StorageStation::new_machine_info(ConstMachine &machine)
{
  /*
  if(machine.state() == "PROCESSED")
  {
    if(!machine.has_instruction_bs())
    {
      printf("machine %s without instructions",name_.c_str());
      return;
    }
    gzwrap::Pose3d spawn_pose;
    if(machine.instruction_bs().side() == llsf_msgs::MachineSide::INPUT)
    {
      spawn_pose = gzwrap::Pose3d(input_x(),input_y(),BELT_HEIGHT+(PUCK_HEIGHT/2),0,0,0);
      printf("spawning puck at input\n");
    }
    else if(machine.instruction_bs().side() == llsf_msgs::MachineSide::OUTPUT)
    {
      spawn_pose = gzwrap::Pose3d(output_x(), output_y(),BELT_HEIGHT+(PUCK_HEIGHT/2),0,0,0);
      printf("spawning puck at output\n");
    }
    else
      spawn_pose = gzwrap::Pose3d::Zero;

    gazsim_msgs::Color spawn_clr;
    switch(machine.instruction_bs().color()){
      case llsf_msgs::BaseColor::BASE_BLACK:
        spawn_clr = gazsim_msgs::Color::BLACK;
        break;
      case llsf_msgs::BaseColor::BASE_SILVER:
        spawn_clr = gazsim_msgs::Color::SILVER;
        break;
      case llsf_msgs::BaseColor::BASE_RED:
      default:
        spawn_clr = gazsim_msgs::Color::RED;
        break;
    }

    spawn_puck(spawn_pose, spawn_clr);
    have_puck_ = "workpiece_base";
    set_state(State::PROCESSED);
    set_state(State::DELIVERED);
  }
  */
}

void StorageStation::init_storage(){
    printf("%s: initializing STORAGE\n",name_.c_str());


}


void StorageStation::on_instruct_machine_msg(ConstInstructMachinePtr &msg){

    //printf("MPS:GOT INSTRUCT MESSAGE\n");

    if (msg->set() != llsf_msgs::INSTRUCT_MACHINE_SS ||
            msg->machine() != name_){
        return;
    }

    std::string machine_name = msg->machine();

    //std::printf("INSTRUCTION MSG FOR: %s\n", machine_name.c_str());

    switch (msg->ss().operation())
    {
    case llsf_msgs::STORE:
        printf("%s: STORE PUCK\n",machine_name.c_str());
        refbox_reply(msg);
        break;
    case llsf_msgs::RETRIEVE:
        printf("%s: RETRIEVE PUCK\n",machine_name.c_str());
        // X and Z is swapped to match message coords with gazebo coords
        retrieve_puck(msg->ss().slot().z(),msg->ss().slot().y(),msg->ss().slot().x());
        refbox_reply(msg);
        break;
    default:
        printf("%s: unknown ss TASK\n",machine_name.c_str());
        break;
    }
}


void StorageStation::on_new_puck(ConstNewPuckPtr &msg)
{

  Mps::on_new_puck(msg);
  physics::ModelPtr new_puck = world_->GZWRAP_MODEL_BY_NAME(msg->puck_name());
  //if(puck_in_input(new_puck->GZWRAP_WORLD_POSE()) || puck_in_output(new_puck->GZWRAP_WORLD_POSE()))
  //{
    have_puck_ = new_puck->GetName();

    for (int z=0; z<SLOT_Z_COUNT;z++)
        for (int y=0; y<SLOT_Y_COUNT; y++)
            for (int x=0; x<SLOT_X_COUNT; x++){
                if (pose_hit(new_puck->GZWRAP_WORLD_POSE(),get_slot_World_position(x,y,z),0.05)){

                    addCap(new_puck,gazsim_msgs::Color::BLACK);

                    store_puck(new_puck->GetName(),x,y,z);
                    return;
                }
            }
    //}
}


void StorageStation::store_puck(std::string puck_name,uint32_t slot_pos_x,uint32_t slot_pos_y,uint32_t slot_pos_z){

   int index = getStorageIndex(slot_pos_x,slot_pos_y,slot_pos_z);

    if (storage_[index] != ""){

        printf("ERROR: SLOT %d,%d,%d not EMPTY stored puck %s\n",slot_pos_x,slot_pos_y,slot_pos_z,storage_[index].c_str());
        return;
    }

    printf("%s: STORING PUCK IN SLOT %d xyz (%d,%d,%d)\n STORAGE CNT: %d\n\n",name_.c_str(),index,slot_pos_x,slot_pos_y,slot_pos_z,++storage_cnt);
    storage_[index] = puck_name;
}

void StorageStation::retrieve_puck(uint32_t slot_pos_x,uint32_t slot_pos_y,uint32_t slot_pos_z){

//    if (puck_in_input(input())){
//    printf("ERROR: INPUT not EMPTY\n");
//        return;
//    }

    int index = getStorageIndex(slot_pos_x,slot_pos_y,slot_pos_z);

    if (storage_[index] == ""){
        printf("ERROR: SLOT %d,%d,%d EMPTY\n",slot_pos_x,slot_pos_y,slot_pos_z);
            return;
    }

    printf("Retrieve PUCK %s FROM SLOT xyz %d,%d,%d\n",storage_[index].c_str(),slot_pos_x,slot_pos_y,slot_pos_z);

    gzwrap::Pose3d pose = input();
    world_->GZWRAP_MODEL_BY_NAME(storage_[index])->SetWorldPose(input());
    printf("%s: Moving PUCK %s to input: %f,%f,%f",name_.c_str(),storage_[index].c_str(),pose.pos.x,pose.pos.y,pose.pos.z);

    storage_[index] = "";
    storage_cnt--;
}


void StorageStation::addCap(physics::ModelPtr puck,gazsim_msgs::Color clr){

    //ADD CAP TO PUCK
    gazsim_msgs::WorkpieceCommand cmd;
    cmd.set_command(gazsim_msgs::Command::ADD_CAP);
    cmd.set_color(clr);
    cmd.set_puck_name(puck->GetName());
    puck_cmd_pub_->Publish(cmd);


    //THIS does not seem to work reliable
    gazebo::msgs::Visual vis_msg;
    vis_msg.set_parent_name(name_+"::body");
    vis_msg.set_name(name_+"::body::have_cap");
    gazebo::msgs::Set(vis_msg.mutable_material()->mutable_diffuse(), gazebo::common::Color(1,1,1));
    visPub_->Publish(vis_msg);


}
// THIS IS STILL EXPERIMENTAL STUFF
void StorageStation::addRing(std::string puck_name, llsf_msgs::RingColor clr,bool active,int number){

    gazsim_msgs::Color color_to_put_ =gazsim_msgs::Color::NONE;

    switch(clr)
    {
      case llsf_msgs::RingColor::RING_BLUE:
        color_to_put_ = gazsim_msgs::Color::BLUE;
        break;
      case llsf_msgs::RingColor::RING_GREEN:
        color_to_put_ = gazsim_msgs::Color::GREEN;
        break;
      case llsf_msgs::RingColor::RING_ORANGE:
        color_to_put_ = gazsim_msgs::Color::ORANGE;
        break;
      case llsf_msgs::RingColor::RING_YELLOW:
        color_to_put_ = gazsim_msgs::Color::YELLOW;
        break;
    }

    gazsim_msgs::WorkpieceCommand cmd;
    cmd.set_command(gazsim_msgs::Command::ADD_RING);
    cmd.set_color(color_to_put_);
    cmd.set_puck_name(puck_name);
    puck_cmd_pub_->Publish(cmd);


    gazebo::msgs::Visual msg;
    msg.set_parent_name(name_+"::body");
    msg.set_name(name_+"::body::base_" + std::to_string(number));
  #if GAZEBO_MAJOR_VERSION > 5
    gazebo::msgs::Set(msg.mutable_pose(), ignition::math::Pose3d(-0.35 + (number*0.11),0,BELT_HEIGHT+0.3,0,0,0));
  #else
    gazebo::msgs::Set(msg.mutable_pose(), gazebo::math::Pose(-0.35 + (number*0.11),0,BELT_HEIGHT+0.3,0,0,0));
  #endif
    if(active)
    {
      msgs::Set(msg.mutable_material()->mutable_diffuse(), gazebo::common::Color(1,0,0));
    }
    else
    {
      msgs::Set(msg.mutable_material()->mutable_diffuse(), gazebo::common::Color(0.3,0,0));
    }
    visPub_->Publish(msg);
}

gzwrap::Pose3d StorageStation::get_slot_World_position(uint32_t slot_x,uint32_t slot_y,uint32_t slot_z){

    double x,y,z;
    slot_z+=1;
    slot_y+=1;
    slot_x+=1;
    x = shelf_pos_x + (slot_x * SLOT_X_OFFSET) +(slot_z *SLOT_Z_OFFSET);
    y = shelf_pos_y + (slot_y * SLOT_Y_OFFSET);
    z = 0.0;

    return gzwrap::Pose3d(gazebo::math::Pose(x,y,z,0,0,0));
}


int StorageStation::getStorageIndex( int x, int y, int z ) {
    return (z * SLOT_X_COUNT * SLOT_Y_COUNT) + (y * SLOT_X_COUNT) + x;
}

//int* StorageStation::to3D( int idx ) {
//    int z = idx / (SLOT_X_COUNT  * SLOT_Y_COUNT );
//    idx -= (z * SLOT_X_COUNT  * SLOT_Y_COUNT );
//    int y = idx / SLOT_X_COUNT ;
//    int x = idx % SLOT_X_COUNT ;
//    int* index = new int[3];
//    index = { x, y, z };
//    return index;
//}
