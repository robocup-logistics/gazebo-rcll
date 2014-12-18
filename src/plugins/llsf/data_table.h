/***************************************************************************
 *  data_table.h - This module stores all logical information
 *    about the llsf simulation (e.g. machine orientations,
 *    light signals, puck locations)
 *
 *  Created: Fri Aug 09 12:22:51 2013
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

#ifndef DATATABLE_H__
#define DATATABLE_H__


#include "refbox_comm.h"
#include <string.h>
#include <llsf_msgs/PuckInfo.pb.h>


#define NUMBER_MACHINES 32
#define NUMBER_PUCKS 44

namespace gazebo
{

  class RefboxComm;

  /** Enum for all available machines
   */  
  typedef enum MachineName
  {
    M1,
    M2,
    M3,
    M4,
    M5,
    M6,
    M7,
    M8,
    M9,
    M10,
    M11,
    M12,
    M13,
    M14,
    M15,
    M16,
    M17,
    M18,
    M19,
    M20,
    M21,
    M22,
    M23,
    M24,
    D1,
    D2,
    D3,
    D4,
    D5,
    D6,
    R1,
    R2,
    NONE
  } MachineName;

  /** Enum for the states a light can have
   */
  typedef enum LightState
  {
    OFF,
    ON,
    BLINK
  } LightState;

  /** Enum for team names
   */
  typedef enum Team
  {
    CYAN,
    MAGENTA,
    NIL
  } Team;

  /** Struct for machine data
   */
  typedef struct Machine
  {
    ///name of the machine (enum)
    MachineName name;
    ///name of the link in the gazebo world
    std::string name_link;
    ///name of the machine (string)
    std::string name_string;
    ///which team the machine belongs to
    Team team;
    ///x coordinate
    double x;
    ///y coordinate
    double y;
    ///orientation of the machine
    double ori;
    ///state of the red light
    LightState red;
    ///state of the yellow light
    LightState yellow;
    ///state of the green light
    LightState green;
   } Machine;

  /** Strunct for puck data
   */
  typedef struct Puck
  {
    ///number of the puck
    int number;
    ///name of the puck ling in the gazebo world
    std::string name_link;
    ///x coordinate
    double x;
    ///y coordinate
    double y;
    /// machine the puck is under
    MachineName under_rfid;
    ///machine of the area the puck is placed in
    MachineName in_machine_area;
    ///state of the puck (S0,P2,FI,...)
    llsf_msgs::PuckState state;
  } Puck;

  /**
   * This class stores all logical information
   *    about the llsf simulation (e.g. machine orientations,
   *    light signals, puck locations)
   */
  class LlsfDataTable
  {
  protected:
    //Constructor (Singleton!)
    LlsfDataTable(physics::WorldPtr world, transport::NodePtr gazebo_node);
  public:
    ///Destructor
    ~LlsfDataTable();
  private:
    ///reference to singleton
    static LlsfDataTable *table_;

  public:
    //get singleton instance (creates one if table_ is null)
    static LlsfDataTable* get_table();
    
    static void init(physics::WorldPtr world, transport::NodePtr gazebo_node);

    //clean everything up
    static void finalize();


    //Getter
    Machine get_machine(MachineName name);
    Machine get_machine(std::string name);
    Machine* get_machines();
    Puck get_puck(int number);

    // Setter
    void set_light_state(MachineName machine, LightState red,
			 LightState yellow, LightState green);
    void set_light_state(std::string machine, LightState red,
			 LightState yellow, LightState green);
    void set_machine_team(std::string machine, Team team);
    void set_puck_pos(int puck, double x, double y);
    void set_puck_under_rfid(int puck, MachineName machine);
    void remove_puck_under_rfid(int puck, MachineName machine);
    void set_puck_in_machine_area(int puck, MachineName machine);
    void set_puck_state(int puck, llsf_msgs::PuckState state);

  private:
    ///Provides communication to the refbox
    RefboxComm *refbox_comm_;
    ///llsf world to look up positions
    physics::WorldPtr world_;


    //data
    Machine machines_[33]; //[MX] gets machine X, except for [0]
    Puck pucks_[NUMBER_PUCKS];

    void init_table();
    void init_machine(MachineName number, std::string name_, std::string name_string);
    void init_puck(int number, std::string name);
  };
}
#endif
