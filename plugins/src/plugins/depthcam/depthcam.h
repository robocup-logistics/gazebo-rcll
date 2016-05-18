/***************************************************************************
 *  depthcam.h - provides depthcam pointcloud in gazebo topic
 *
 *  Created: Wed Nov 25 14:15:39 2015
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

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/transport/transport.hh>
#include <list>
#include <map>
#include <string>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>
#include <configurable/configurable.h>

namespace gazebo
{
  /**
   * Provides depthcam pointcloud in gazebo topic
   * @author Frederik Zwilling
   */
  class DepthCam : public SensorPlugin, public gazebo_rcll::ConfigurableAspect
  {
  public:
    DepthCam();
   ~DepthCam();

    //Overridden ModelPlugin-Functions
    virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/);
    virtual void Reset();

    virtual void OnNewDepthFrame(const float *_image,
    				 unsigned int _width, unsigned int _height,
    				 unsigned int _depth, const std::string &_format);

    virtual void OnNewRGBPointCloud(const float *_pcd,
    				    unsigned int _width, unsigned int _height,
    				    unsigned int _depth, const std::string &_format);

    virtual void OnNewImageFrame(const unsigned char *_image,
    				 unsigned int _width, unsigned int _height,
    				 unsigned int _depth, const std::string &_format);

  private:
    ///Node for communication to fawkes
    transport::NodePtr node_;
    ///Node for communication in gazebo
    transport::NodePtr world_node_;

    transport::PublisherPtr pcl_pub_;

    ///name of the communication channel and the sensor
    std::string name_;

    //config values:
    std::string pcl_topic_;

    unsigned int width_, height_, depth_;
    std::string format_;

    //Depth camera stuff:
    sensors::DepthCameraSensorPtr parentSensor;
    rendering::DepthCameraPtr depthCamera;

    event::ConnectionPtr newDepthFrameConnection;
    event::ConnectionPtr newRGBPointCloudConnection;
    event::ConnectionPtr newImageFrameConnection;

  };
}
