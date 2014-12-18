/***************************************************************************
 *  refbox_comm.h - responsable for the communication with the refbox
 *                  reads/writes the data in the table
 *
 *  Created: Wed Aug 14 21:13:00 2013
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

#ifndef REFBOX_COMM_H__
#define REFBOX_COMM_H__

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/transport/transport.hh>

#include "data_table.h"

#include <llsf_msgs/MachineInfo.pb.h>


typedef const boost::shared_ptr<llsf_msgs::MachineInfo const> ConstMachineInfoPtr;
typedef const boost::shared_ptr<llsf_msgs::PuckInfo const> ConstPuckInfoPtr;

namespace gazebo
{

  class LlsfDataTable;
  struct Machine;
  struct Puck;

  /**
   *  The class RefboxComm is responsable for the communication with the refbox.
   *  It reads/writes the data in the table.
   *
   *  For comunication with the refbox, it uses the Protobuf-Adapter in Fawkes
   */
  class RefboxComm
  {
  public:
    //Constructor
    RefboxComm(LlsfDataTable *table, transport::NodePtr gazebo_node);
    ///Destructor
    ~RefboxComm();

    //send protobuf msg for puck placed under rfid to fawkes and refbox
    void send_puck_placed_under_rfid(int puck, Machine & machine);
    //send protobuf msg for puck removed under rfid to fawkes and refbox
    void send_remove_puck_from_machine(int puck, Machine & machine);


  private:
    ///Pointer to the data table
    LlsfDataTable *table_;
    ///Pointer to the communication node from gazebo
    transport::NodePtr gazebo_node_;

    ///Publisher for communication to the refbox (via the adapter)
    transport::PublisherPtr place_puck_under_machine_pub_;
    transport::PublisherPtr remove_puck_from_machine_pub_;

    ///Suscriber for MachineInfos from the refbox
    transport::SubscriberPtr machine_info_sub_;
    transport::SubscriberPtr puck_info_sub_;

    void on_machine_info_msg(ConstMachineInfoPtr &msg);
    void on_puck_info_msg(ConstPuckInfoPtr &msg);
  };
}
#endif
