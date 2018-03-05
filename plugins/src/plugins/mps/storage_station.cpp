/***************************************************************************
 *  storage_station.cpp - controls a storagestation mps
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
    storage_ = new Storage[STORAGE_SIZE];

    //EMPTY all storage slots
    for (int i=0;i< STORAGE_SIZE; i++){
        storage_[i].puck_name ="";
        storage_[i].has_puck =false;
    }

    pucks_spawned = false;

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




        init_storage();



//    for (int z=0; z<SLOT_Z_COUNT;z++){
//        for (int y=0; y<SLOT_Y_COUNT; y++)
//            for (int x=0; x<SLOT_X_COUNT; x++){
//            spawn_puck(get_slot_World_position(x,y,z),gazsim_msgs::Color::RED);
//            }
//    }
}

StorageStation::~StorageStation(){

    delete storage_;
}

void StorageStation::OnUpdate(const common::UpdateInfo & info){
 Mps::OnUpdate(info);
    if(model_->GetWorld()->GZWRAP_SIM_TIME().Double() - created_time_ >10 &&  !pucks_spawned){
        printf("%s: Start Spawning Pucks\n",name_.c_str());


        for (uint i =0; i< STORAGE_SIZE; i++){

            if (storage_[i].has_puck){

                Storage slot =storage_[i];
                std::string puck_name = spawn_puck(get_slot_World_position(slot.slot_x,slot.slot_y,slot.slot_z),slot.base_clr);

                if(puck_name == ""){
                    printf("%s: ERROR SPAWN of STORAGE PUCK FAILED\n",name_.c_str());
                    continue;
                }
                storage_[i].puck_name = puck_name;
            }
        }
        pucks_spawned = true;
    }


}


void StorageStation::init_storage(){

    std::vector<std::string> slotIds = config->get_strings("plugins/mps/storage-station/slots");

    for (uint i =0; i< slotIds.size(); i++){
        std::string slotpos = slotIds.at(i);
        printf("%s: generating puck for slot %s\n",name_.c_str(),slotpos.c_str());
        std::string configname = "plugins/mps/storage-station/slot";
        configname.append(slotIds.at(i));
        std::vector<std::string> puck_cfg = config->get_strings(configname.c_str());

        int x = (int)slotpos.at(0)-'0';
        int y = (int)slotpos.at(1)-'0';
        int z = (int)slotpos.at(2)-'0';
        int index = getStorageIndex(x,y,z);

        storage_[index].slot_x =x;
        storage_[index].slot_y =y;
        storage_[index].slot_z =z;

        if (puck_cfg.at(0) == "BLACK") {storage_[index].base_clr = gazsim_msgs::Color::BLACK;}
        else if (puck_cfg.at(0) == "SILVER") {storage_[index].base_clr = gazsim_msgs::Color::SILVER;}
        else if (puck_cfg.at(0) == "RED") {storage_[index].base_clr = gazsim_msgs::Color::RED;}
        else printf("%s: unknown base color %s at slot%s \n",name_.c_str(),puck_cfg.at(0).c_str(),slotpos.c_str());
         int last = puck_cfg.size()-1;

         if (last > 1){
             //add Rings to puck
             for (int j = 1; j< last; j++){

                 if (puck_cfg.at(j) == "YELLOW") {storage_[index].ring_colors.push_back(gazsim_msgs::Color::YELLOW);}
                 else if (puck_cfg.at(j) == "ORANGE") {storage_[index].ring_colors.push_back(gazsim_msgs::Color::ORANGE);}
                 else if (puck_cfg.at(j) == "GREEN") {storage_[index].ring_colors.push_back(gazsim_msgs::Color::GREEN);}
                 else if (puck_cfg.at(j) == "BLUE") {storage_[index].ring_colors.push_back(gazsim_msgs::Color::BLUE);}
                 else printf("%s: ERROR unknown ring color %s at slot%s \n",name_.c_str(),puck_cfg.at(j).c_str(),slotpos.c_str());
             }
         }

         if (puck_cfg.at(last) == "BLACK") {storage_[index].cap_clr  = gazsim_msgs::Color::BLACK;}
         else if (puck_cfg.at(last) == "GRAY") {storage_[index].cap_clr= gazsim_msgs::Color::GREY;}
         else if (puck_cfg.at(last) == "") {storage_[index].cap_clr = gazsim_msgs::Color::NONE;}
         else printf("%s: ERROR unknown cap color %s at slot%s \n",name_.c_str(),puck_cfg.at(last).c_str(),slotpos.c_str());


         storage_[index].has_puck = true;
    }
}


void StorageStation::on_puck_msg(ConstPosePtr &msg)
{
  if(msg->name() == puck_on_conveyor && !puck_in_input(msg))
  {
    puck_on_conveyor ="";
    set_state(State::RETRIEVED);
  } 
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
        retrieve_puck(msg->ss().slot().x(),msg->ss().slot().y(),msg->ss().slot().z());
        refbox_reply(msg);
        break;
    default:
        printf("%s: unknown ss TASK\n",machine_name.c_str());
        break;
    }
}


void StorageStation::on_new_puck(ConstNewPuckPtr &msg)
{

  //if(puck_in_input(new_puck->GZWRAP_WORLD_POSE()) || puck_in_output(new_puck->GZWRAP_WORLD_POSE()))
  //{


  //    for (int z=0; z<SLOT_Z_COUNT;z++)
  //        for (int y=0; y<SLOT_Y_COUNT; y++)
  //            for (int x=0; x<SLOT_X_COUNT; x++){
  //                if (pose_hit(new_puck->GZWRAP_WORLD_POSE(),get_slot_World_position(x,y,z),0.05)){

  for ( uint i = 0; i< STORAGE_SIZE; i++){
         if ( storage_[i].puck_name == msg->puck_name()){
             Storage slot = storage_[i];
             Mps::on_new_puck(msg);
             physics::ModelPtr new_puck = world_->GZWRAP_MODEL_BY_NAME(msg->puck_name());

             addRings(slot.puck_name,slot.ring_colors);
             addCap(new_puck,slot.cap_clr);

             store_puck(new_puck->GetName(),slot.slot_x,slot.slot_y,slot.slot_z);
                    return;
                }
            }
    //}
}


void StorageStation::store_puck(std::string puck_name,uint32_t slot_pos_x,uint32_t slot_pos_y,uint32_t slot_pos_z){

   int index = getStorageIndex(slot_pos_x,slot_pos_y,slot_pos_z);

   if (storage_[index].puck_name == puck_name){
       // puck is already in Storage
       return;
    }


    if (storage_[index].puck_name != ""){

        printf("%s ERROR: SLOT %d,%d,%d not EMPTY stored puck %s\n",name_.c_str(),slot_pos_x,slot_pos_y,slot_pos_z,storage_[index].puck_name.c_str());
        return;
    }

    printf("%s: STORING PUCK IN SLOT %d xyz (%d,%d,%d)\n STORAGE CNT: %d\n\n",name_.c_str(),index,slot_pos_x,slot_pos_y,slot_pos_z,++storage_cnt);
    storage_[index].puck_name = puck_name;
}

void StorageStation::retrieve_puck(uint32_t slot_pos_x,uint32_t slot_pos_y,uint32_t slot_pos_z){

//    if (puck_in_input(input())){
//    printf("ERROR: INPUT not EMPTY\n");
//        return;
//    }
    // X and Z is swapped to match message coords with gazebo coords
    int index = getStorageIndex(slot_pos_x,slot_pos_y,slot_pos_z);

    if (storage_[index].puck_name == ""){
        printf("%s ERROR: SLOT %d,%d,%d EMPTY\n",name_.c_str(),slot_pos_x,slot_pos_y,slot_pos_z);
            return;
    }

    printf("Retrieve PUCK %s FROM SLOT xyz %d,%d,%d\n",storage_[index].puck_name.c_str(),slot_pos_x,slot_pos_y,slot_pos_z);

    gzwrap::Pose3d pose = input();
    world_->GZWRAP_MODEL_BY_NAME(storage_[index].puck_name)->SetWorldPose(input());
    printf("%s: Moving PUCK %s to input: %f,%f,%f",
           name_.c_str(),
           storage_[index].puck_name.c_str(),
           pose.GZWRAP_POS.GZWRAP_X,
           pose.GZWRAP_POS.GZWRAP_Y,
           pose.GZWRAP_POS.GZWRAP_Z);

    puck_on_conveyor = storage_[index].puck_name;

    storage_[index].puck_name = "";
    storage_[index].has_puck = false;
    storage_cnt--;

    set_state(State::PROCESSED);
    set_state(State::DELIVERED);

}


void StorageStation::addCap(physics::ModelPtr puck,gazsim_msgs::Color clr){

    //ADD CAP TO PUCK
    gazsim_msgs::WorkpieceCommand cmd;
    cmd.set_command(gazsim_msgs::Command::ADD_CAP);
    cmd.add_color(clr);
    cmd.set_puck_name(puck->GetName());
    puck_cmd_pub_->Publish(cmd);

}
// THIS IS STILL EXPERIMENTAL STUFF
void StorageStation::addRings(std::string puck_name, std::vector<gazsim_msgs::Color> clr_list){


    for (uint i=0 ; i < clr_list.size(); i++ ){
        gazsim_msgs::WorkpieceCommand cmd;
        cmd.set_command(gazsim_msgs::Command::ADD_RING);
        cmd.add_color(clr_list.at(i));
        cmd.set_puck_name(puck_name);
        puck_cmd_pub_->Publish(cmd);
    }


}

gzwrap::Pose3d StorageStation::get_slot_World_position(uint32_t slot_x,uint32_t slot_y,uint32_t slot_z){

    double x,y,z;
    slot_z+=1;
    slot_y+=1;
    slot_x+=1;
    x = shelf_pos_x + (slot_x * SLOT_X_OFFSET) +(slot_z *SLOT_Z_OFFSET);
    y = shelf_pos_y + (slot_y * SLOT_Y_OFFSET);
    z = PUCK_HEIGHT*0.5;

    return gzwrap::Pose3d(x,y,z,0,0,0);
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
