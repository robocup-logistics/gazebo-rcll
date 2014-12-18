/***************************************************************************
 *  gripper_laser_sensor.cpp - simulates a laser sensor at the gripper
 *                         to detect the machines
 *
 *  Created: Fri Aug 30 17:09:59 2013
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

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <gazebo/transport/transport.hh>
#include "simDevice.h"
#include "gripper_laser_sensor.h"
#include "../../msgs/Float.pb.h"

using namespace gazebo;


/** Constructor
 * @param model Model to the gazebo model of the plugin
 * @param node Transport node to publish and subscribe messages on
 * @param sensorPtr Pointer to the laser sensor of the model
 * @param side Is this the sensor on the left or right side?
 */
GripperLaserSensor::GripperLaserSensor(physics::ModelPtr model, transport::NodePtr node, sensors::SensorPtr sensorPtr, Side side)
 : SimDevice(model, node)
{
  this->parent_sensor_ = boost::dynamic_pointer_cast<sensors::RaySensor>(sensorPtr);
  side_ = side;
}
GripperLaserSensor::~GripperLaserSensor()
{
}

void GripperLaserSensor::init()
{
  printf("Initialize GripperLaserSensor \n");

  //Register OnNewLaserScans function
  this->new_laser_scans_connection_ = this->parent_sensor_->GetLaserShape()->ConnectNewLaserScans(boost::bind(&GripperLaserSensor::on_new_laser_scans, this));
}

void GripperLaserSensor::create_publishers()
{
  switch(side_)
  {
  case LEFT:
    {
      this->gripper_laser_sensor_pub_ = this->node->Advertise<gazsim_msgs::Float>("~/RobotinoSim/GripperLaserSensor/Left/");
      break;
    }
  case RIGHT:
    {
      this->gripper_laser_sensor_pub_ = this->node->Advertise<gazsim_msgs::Float>("~/RobotinoSim/GripperLaserSensor/Right/");
      break;
    }
  }
}

void GripperLaserSensor::create_subscribers()
{
}

void GripperLaserSensor::update()
{
  //sending the laser scans happens in OnNewLaserScans()
}

void GripperLaserSensor::on_new_laser_scans()
{
  if(gripper_laser_sensor_pub_->HasConnections())
  {
    //Get relevant data
    //int numRays = parent_sensor_->GetRangeCount();
    //float angleMin = parent_sensor_->GetAngleMin().Radian();
    //float angleMax = parent_sensor_->GetAngleMax().Radian();
    //float angleStep = parent_sensor_->GetAngleResolution();
    //float rangeMin = parent_sensor_->GetRangeMin();
    //float rangeMax = parent_sensor_->GetRangeMax();
    

    //create Protobuf message
    gazsim_msgs::Float laserMsg;
    //laser has only one beam
    laserMsg.set_value(parent_sensor_->GetRange(0));
    
    //send message
    gripper_laser_sensor_pub_->Publish(laserMsg);
  }
}
