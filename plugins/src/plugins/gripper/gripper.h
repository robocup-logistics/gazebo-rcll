/***************************************************************************
 *  gripper.cpp - provides gripper simulation
 *
 *  Created: Mon Mar 23 2015
 *  Copyright  2015 Stefan Profanter
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
#include <list>
#include <string.h>
#include <fnmatch.h>

#include <boost/thread/mutex.hpp>
#include <configurable/configurable.h>

#include <random>


//config values
#define TOPIC_SET_GRIPPER config->get_string("plugins/gripper/topic-set-gripper").c_str()
#define TOPIC_HOLDS_PUCK config->get_string("plugins/gripper/topic-holds-puck").c_str()
#define TOPIC_JOINT config->get_string("plugins/gripper/topic-joint").c_str()
#define RADIUS_GRAB_AREA config->get_float("plugins/gripper/radius-grab-area")

//probability of puck falling down (per second)
#define PROB_PUCK_FALLS config->get_float("plugins/gripper/prob-puck-falls")
//probability of fail during pick up of the puck
#define PROB_FAILING_PICK_UP config->get_float("plugins/gripper/prob-failing-pick-up")

//the following parameters are only used when the floor clean plugin is off, so gripper itself needs to remove randomly fallen pucks
#define FLOOR_CLEAN_OFF config->get_bool("plugins/floor-clean/floor-clean-off")
#define FIELD_X_SIZE config->get_float("plugins/floor-clean/field-x-size")
#define FIELD_Y_SIZE config->get_float("plugins/floor-clean/field-y-size")
#define PUCK_HEIGHT config->get_float("plugins/mps/puck_height");
#define PUCK_SIZE config->get_float("plugins/mps/puck_size");


enum ActionOnUpdate{
  NOTHING = 0,
  OPEN = 1,
  CLOSE = 2
} typedef ActionOnUpdate;

namespace gazebo
{
  /**
   * Provides gripper simulation
   * @author Stefan Profanter
   */
  class Gripper : public ModelPlugin, public gazebo_rcll::ConfigurableAspect
  {
  public:
    Gripper();
    ~Gripper();

    //Overridden ModelPlugin-Functions
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
    virtual void OnUpdate(const common::UpdateInfo &);
    virtual void Reset();

  private:
    /// Pointer to the gazbeo model
    physics::ModelPtr model_;
    physics::ModelPtr robotino_;
    physics::LinkPtr robotino_link_;
    /// Pointer to the update event connection
    event::ConnectionPtr update_connection_;
    ///Node for communication to fawkes
    transport::NodePtr node_;
    ///name of the gps and the communication channel
    std::string name_;

    physics::ModelPtr grippedPuck;

    //Gripper Stuff:

    ///Set gripper callback
    void on_set_gripper_msg(ConstIntPtr &msg);

    ///Suscriber for SetGripper
    transport::SubscriberPtr set_gripper_sub_;
    /// Publisher for has_puck
    gazebo::transport::PublisherPtr has_puck_pub_;

    /// Publisher to announce which puck is hold by the gripper
    gazebo::transport::PublisherPtr joint_pub_;

    gazebo::physics::JointPtr grabJoint;

    static gazebo::physics::LinkPtr getLinkEndingWith(physics::ModelPtr model, std::string link);
    static gazebo::physics::JointPtr getJointEndingWith(physics::ModelPtr model, std::string link);

    void close();
    void open();

    void setPuckPose();
    void sendHasPuck(bool has_puck);

    gazebo::physics::ModelPtr getNearestPuck();

    ActionOnUpdate last_action_rcvd_;

    gazebo::physics::LinkPtr getGripperLink();

    // Function testing whether event with probability per time p happens
    // To minimize computing cost, approximation for small absolute probability p^delta_t
    // is used, so use carefully
    // @param p probability per second
    // @param delta_t timestep in which the happening of event should be calculated
    bool test_probability(double p, double delta_t);
    // Function testing whether event with probability p happens
    // @param p probability
    bool test_probability(double p);
    common::Time oldTime_ = 0;
    std::uniform_real_distribution<double> do_test_;
    std::mt19937 rnd_gen_;

    //Position to put the wasted pucks
    void setPuckPoseOffField(physics::ModelPtr puck);
    double x_to_put_;
    double y_to_put_;
    double z_to_put_;
    
  };
}
