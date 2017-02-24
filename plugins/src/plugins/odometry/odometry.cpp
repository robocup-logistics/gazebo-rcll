/***************************************************************************
 *  odometry.cpp - provides odometry simulation of object model
 *
 *  Created: Fri Feb 20 18:05:07 2015
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

#include <math.h>

#include <utils/misc/gazebo_api_wrappers.h>

#include "odometry.h"

using namespace gazebo;

// Register this plugin to make it available in the simulator
GZ_REGISTER_MODEL_PLUGIN(Odometry)

///Constructor
Odometry::Odometry()
{
}
///Destructor
Odometry::~Odometry()
{
  printf("Destructing Odometry Plugin!\n");
}

/** on loading of the plugin
 * @param _parent Parent Model
 */
void Odometry::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) 
{
    // Store the pointer to the model
    this->model_ = _parent;


    estimate_x = 0;
    estimate_y = 0;
    estimate_omega = 0;

    last_sent_time_ = model_->GetWorld()->GZWRAP_SIM_TIME().Double();

    //get the model-name
    this->name_ = model_->GetName();
    printf("Loading Odometry Plugin of model %s\n", name_.c_str());

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&Odometry::OnUpdate, this, _1));

    //Create the communication Node for communication with fawkes
    this->node_ = transport::NodePtr(new transport::Node());

    //the namespace is set to the model name!
    this->node_->Init(model_->GetWorld()->GZWRAP_NAME()+"/"+name_);

    //init last sent time
    last_sent_time_ = model_->GetWorld()->GZWRAP_SIM_TIME().Double();

    //create publisher
    this->odometry_pub_ = this->node_->Advertise<msgs::Vector3d>("~/RobotinoSim/Odometry/");

    //create subscriber
    this->set_odometry_sub_ = this->node_->Subscribe(std::string("~/RobotinoSim/SetOdometry/"), &Odometry::on_set_odometry_msg, this);
}


/** Called by the world update start event
 */
void Odometry::OnUpdate(const common::UpdateInfo & /*_info*/)
{
    //Send position information to Fawkes
    double time = model_->GetWorld()->GZWRAP_SIM_TIME().Double();
    if(time - last_sent_time_ > (1.0 / 10.0))
    {
        send_position();
        last_sent_time_ = time;
    }
}

/** on Gazebo reset
 */
void Odometry::Reset()
{
}

/** Functions for recieving Messages (registerd via suscribers)
 * @param msg message
 */
void Odometry::on_set_odometry_msg(ConstVector3dPtr &msg)
{
	//std::cout << "Got new odometry: " << msg->x() << "|" << msg->y() << "|" << msg->z() << std::endl;
	{
		boost::mutex::scoped_lock lock(readingsMutex);
		estimate_x = msg->x();
		estimate_y = msg->y();
		estimate_omega = msg->z();
		last_sent_time_ = model_->GetWorld()->GZWRAP_SIM_TIME().Double();
	}
}


/** Sending position to Fawkes
 * 
 */
void Odometry::send_position()
{
    gzwrap::Vector3d linearVel = this->model_->GZWRAP_RELATIVE_LINEAR_VEL();
    gzwrap::Vector3d angularVel = this->model_->GZWRAP_RELATIVE_ANGULAR_VEL();

    //get the elapsed time since last update
    double elapsedSeconds = model_->GetWorld()->GZWRAP_SIM_TIME().Double()-last_sent_time_;

    //now add some simulated error
    // rand multiplied by max noise which is in seconds
    elapsedSeconds += ((rand() % 2001 - 1000) / 1000.0)*0.1;

    // we need to project the velocity values from robot base to world base
    float vecX = linearVel.GZWRAP_X;
    float vecY = linearVel.GZWRAP_Y;

    //rotate vector by current robot angle
    float cs = cos(estimate_omega);
    float sn = sin(estimate_omega);
    float tx = vecX * cs - vecY * sn;
    float ty = vecX * sn + vecY * cs;

    // now update robot's position
	{
		boost::mutex::scoped_lock lock(readingsMutex);
		estimate_x += tx * elapsedSeconds;
		estimate_y += ty * elapsedSeconds;
		estimate_omega += angularVel.GZWRAP_Z * elapsedSeconds;
		if (estimate_omega < -M_PI)
			estimate_omega = 2 * M_PI - estimate_omega;
		else if (estimate_omega > M_PI)
			estimate_omega = -2 * M_PI + estimate_omega;
	}

    if(odometry_pub_->HasConnections())
    {
        //build message
        msgs::Vector3d posMsg;
        posMsg.set_x(estimate_x);
        posMsg.set_y(estimate_y);
        posMsg.set_z(estimate_omega);

        //send
        odometry_pub_->Publish(posMsg);
    }
}
