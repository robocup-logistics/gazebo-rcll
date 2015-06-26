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


//config values
#define TOPIC_SET_GRIPPER "~/RobotinoSim/SetGripper/"
#define TOPIC_HOLDS_PUCK "~/RobotinoSim/GripperHasPuck/"
#define TOPIC_JOINT "/GripperJoints/Holding"
#define RADIUS_GRAB_AREA 0.05


namespace gazebo
{
    /**
    * Provides gripper simulation
    * @author Stefan Profanter
    */
    class Gripper : public ModelPlugin
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
    };
}
