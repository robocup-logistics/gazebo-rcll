/***************************************************************************
 *  llsf_refbox_comm.h - World plugin for the refbox connection in the llsf
 *
 *  Created: Fri Mar 06 16:09:42 2015
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
#include <protobuf_comm/client.h>
#include <protobuf_comm/message_register.h>

#include <llsf_msgs/MachineInfo.pb.h>
#include <llsf_msgs/MachineCommands.pb.h>
#include <llsf_msgs/SimTimeSync.pb.h>
#include <llsf_msgs/GameState.pb.h>
#include <llsf_msgs/GameInfo.pb.h>
#include <gazsim_msgs/SimTime.pb.h>
#include <llsf_msgs/OrderInfo.pb.h>
#include <configurable/configurable.h>

//typedefs for sending the messages over the gazebo node
typedef const boost::shared_ptr<llsf_msgs::MachineInfo const> ConstMachineInfoPtr;
typedef const boost::shared_ptr<gazsim_msgs::SimTime const> ConstSimTimePtr;
typedef const boost::shared_ptr<llsf_msgs::SetGameState const> ConstSetGameStatePtr;
typedef const boost::shared_ptr<llsf_msgs::SetGamePhase const> ConstSetGamePhasePtr;
typedef const boost::shared_ptr<llsf_msgs::SetTeamName const> ConstSetTeamNamePtr;
typedef const boost::shared_ptr<llsf_msgs::SetMachineState const> ConstSetMachineStatePtr;
typedef const boost::shared_ptr<llsf_msgs::MachineAddBase const> ConstMachineAddBasePtr;
typedef const boost::shared_ptr<llsf_msgs::SetOrderDeliveredByColor const> ConstSetOrderDeliveredByColorPtr;


//config values
#define PROTO_DIR config->get_string("plugins/llsf-refbox-comm/proto-dir").c_str()
#define REFBOX_HOST config->get_string("plugins/llsf-refbox-comm/refbox-host").c_str()
#define REFBOX_PORT config->get_int("plugins/llsf-refbox-comm/refbox-port")
#define RECONNECT_INTERVAL config->get_int("plugins/llsf-refbox-comm/reconnect-interval") //in s
//Max number of reconnect attempts (due to crash when tried to connect often)
#define RECONNECT_ATTEMPTS config->get_int("plugins/llsf-refbox-comm/reconnect-attempts")
#define TOPIC_MACHINE_INFO config->get_string("plugins/llsf-refbox-comm/topic-machine-info").c_str()
#define TOPIC_GAME_STATE config->get_string("plugins/llsf-refbox-comm/topic-game-state").c_str()
#define TOPIC_TIME config->get_string("plugins/llsf-refbox-comm/topic-time").c_str()
#define TOPIC_SET_GAME_STATE config->get_string("plugins/llsf-refbox-comm/topic-set-game-state").c_str()
#define TOPIC_SET_GAME_PHASE config->get_string("plugins/llsf-refbox-comm/topic-set-game-phase").c_str()
#define TOPIC_SET_TEAM_NAME config->get_string("plugins/llsf-refbox-comm/topic-set-team-name").c_str()
#define TOPIC_SET_MACHINE_STATE config->get_string("plugins/llsf-refbox-comm/topic-set-machine-state").c_str()
#define TOPIC_MACHINE_ADD_BASE config->get_string("plugins/llsf-refbox-comm/topic-machine-add-base").c_str()
#define TOPIC_SET_ORDER_DELIVERY_BY_COLOR config->get_string("plugins/llsf-refbox-comm/topic-set-order-delivery-by-color").c_str()

namespace protobuf_comm {
  class ProtobufStreamClient;
}

namespace gazebo
{
  /**
   * World plugin for the refbox connection in the llsf
   * Basically it gets the peer msgs from the refbox and publishes these msgs in gazebo
   */
  class LlsfRefboxCommPlugin : public WorldPlugin, public gazebo_rcll::ConfigurableAspect
  {
  public:
    ///Constructor
    LlsfRefboxCommPlugin();
    ///Destructor
    ~LlsfRefboxCommPlugin();

    virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    ///on refbox connection established
    void client_connected();
    ///on refbox connection lost
    void client_disconnected(const boost::system::error_code &error);
    ///on message from the refbox
    void client_msg(uint16_t comp_id, uint16_t msg_type,
		    std::shared_ptr<google::protobuf::Message> msg);

  private:
    ///update function
    void Update();
    event::ConnectionPtr update_connection_;

    ///Node for communication
    transport::NodePtr node_;
    physics::WorldPtr world_;

    protobuf_comm::ProtobufStreamClient *client_;
    std::vector<std::string> proto_dirs_;
    protobuf_comm::MessageRegister      *message_register_;

    //Publisher and subscriber for the connection to gazebo
    gazebo::transport::PublisherPtr machine_info_pub_;
    gazebo::transport::PublisherPtr game_state_pub_;
    /* gazebo::transport::SubscriberPtr place_puck_under_machine_sub_; */
    /* gazebo::transport::SubscriberPtr remove_puck_under_machine_sub_; */
    gazebo::transport::SubscriberPtr time_sync_sub_;
    gazebo::transport::SubscriberPtr set_game_state_sub_;
    gazebo::transport::SubscriberPtr set_game_phase_sub_;
    gazebo::transport::SubscriberPtr set_team_name_sub_;
    gazebo::transport::SubscriberPtr set_machine_state_sub_;
    gazebo::transport::SubscriberPtr machine_add_base_sub_;
    gazebo::transport::SubscriberPtr set_order_deliverd_by_color_sub_;

    //handler methods
    /* void on_puck_place_msg(ConstPlacePuckUnderMachinePtr &msg); */
    /* void on_puck_remove_msg(ConstRemovePuckFromMachinePtr &msg); */
    void on_time_sync_msg(ConstSimTimePtr &msg);
    void on_set_game_state_msg(ConstSetGameStatePtr &msg);
    void on_set_game_phase_msg(ConstSetGamePhasePtr &msg);
    void on_set_team_name_msg(ConstSetTeamNamePtr &msg);
    void on_set_machine_state_msg(ConstSetMachineStatePtr &msg);
    void on_machine_add_base_msg(ConstMachineAddBasePtr &msg);
    void on_set_order_delvered_by_color_msg(ConstSetOrderDeliveredByColorPtr &msg);

    //helper variables
    bool connected_;
    double last_connect_try_;
    int connect_tries_;

    void create_client();
    
  };
  GZ_REGISTER_WORLD_PLUGIN(LlsfRefboxCommPlugin)
}
