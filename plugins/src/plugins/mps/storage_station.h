/***************************************************************************
 *  storage_station.h - controls a storagestation mps
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

#ifndef STORAGESTATION_H
#define STORAGESTATION_H

#include "mps.h"
#include "../puck/puck.h"

/// number of slots and shelfs
#define SLOT_X_COUNT 2
#define SLOT_Y_COUNT 4
#define SLOT_Z_COUNT 6
#define STORAGE_SIZE SLOT_Z_COUNT * SLOT_Y_COUNT * SLOT_X_COUNT
#define SHELF_POS_X 10
#define SHELF_POS_Y 10
#define SHELF_POS_Z 0


/// offsets for calculation of world pose slot positions
#define SLOT_X_OFFSET 0.1
#define SLOT_Y_OFFSET 0.2
#define SLOT_Z_OFFSET 0.4


namespace gazebo
{

class Storage;

class StorageStation : public Mps
{
public:
  StorageStation(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  ~StorageStation();
private:
  void on_puck_msg(ConstPosePtr &msg);
  void new_machine_info(ConstMachine &machine);
  void on_instruct_machine_msg(ConstInstructMachinePtr &msg);
  void OnUpdate(const common::UpdateInfo &info);
  
  void on_new_puck(ConstNewPuckPtr &msg);

  gzwrap::Pose3d get_slot_World_position(uint32_t slot_x, uint32_t slot_y, uint32_t slot_z);
  void init_storage();
  void store_puck(std::__cxx11::string puck_name, uint32_t slot_pos_x, uint32_t slot_pos_y, uint32_t slot_pos_z);
  void retrieve_puck(uint32_t slot_pos_x, uint32_t slot_pos_y, uint32_t slot_pos_z);

  void addCap(physics::ModelPtr puck,gazsim_msgs::Color clr);
  void addRings(std::__cxx11::string puck_name, std::vector<gazsim_msgs::Color> clr_list);

  int getStorageIndex( int x, int y, int z );
  int* to3D( int idx );
  /// position of the first shelf first slot logic coords: (0,0,0)
  double shelf_pos_x;
  double shelf_pos_y;
  double shelf_pos_z;

  /// offset to reach each consecutive slot. Multiply these n times the logic slot
  double shelf_x_offset;
  double shelf_y_offset;
  double shelf_z_offset;

  Storage* storage_;
  bool pucks_spawned;

    //not really needed just for testing
  int storage_cnt;

  std::string  puck_on_conveyor;


};

class Storage{
public:
    std::string puck_name;
    int slot_x;
    int slot_y;
    int slot_z;
    bool has_puck;
    gazsim_msgs::Color base_clr;
    std::vector<gazsim_msgs::Color> ring_colors;
    gazsim_msgs::Color cap_clr;
};

}
#endif // STORAGETATION_H
