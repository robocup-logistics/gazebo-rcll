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

#include "depthcam.h"

#include <fnmatch.h>
#include <math.h>
#include <memory>
#include <vector>

using namespace gazebo;

// Register this plugin to make it available in the simulator
GZ_REGISTER_SENSOR_PLUGIN(DepthCam)

///Constructor
DepthCam::DepthCam() : SensorPlugin(), width_(0), height_(0), depth_(0), format_("")
{
}
///Destructor
DepthCam::~DepthCam()
{
	printf("Destructing DepthCam Plugin!\n");
	parentSensor.reset();
	depthCamera.reset();
}

/** on loading of the plugin
 * @param _parent Parent Model
 */
void
DepthCam::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
	// DepthCameraPlugin::Load(_sensor, _sdf);
	//get the model-name
#if GAZEBO_MAJOR_VERSION >= 7
	name_ = _sensor->Name();
#else
	name_ = _sensor->GetName();
#endif
	printf("DepthCam: Loading plugin of sensor %s\n", name_.c_str());

	//Create the communication Node for communication with fawkes
	node_ = transport::NodePtr(new transport::Node());
	//the namespace is set to the model name!
#if GAZEBO_MAJOR_VERSION >= 7
	node_->Init(_sensor->WorldName() + "/" + name_);
#else
	node_->Init(_sensor->GetWorldName() + "/" + name_);
#endif

	//Create the communication Node in gazbeo
	world_node_ = transport::NodePtr(new transport::Node());
	//the namespace is set to the world name!
#if GAZEBO_MAJOR_VERSION >= 7
	world_node_->Init(_sensor->WorldName());
#else
	world_node_->Init(_sensor->GetWorldName());
#endif

	//read config values
	pcl_topic_ = config->get_string("plugins/depthcam/topic-pcl");

	//create publisher
	pcl_pub_ = node_->Advertise<msgs::PointCloud>(pcl_topic_.c_str());

	//Adding those 2 lines enables, that the compiler uses the correct
	//one. Gazebo uses boost::shared_ptr up to version 5.2.1. Since
	//version 5.3.0 it uses std::shared_ptr.
	using boost::dynamic_pointer_cast;
#if __cplusplus >= 201103L
	using std::dynamic_pointer_cast;
#endif
	parentSensor = dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor);
#if GAZEBO_MAJOR_VERSION >= 7
	depthCamera = parentSensor->DepthCamera();
#else
	depthCamera = parentSensor->GetDepthCamera();
#endif

	if (!parentSensor) {
		gzerr << "DepthCam Plugin not attached to a depthCamera sensor\n";
		return;
	}

#if GAZEBO_MAJOR_VERSION >= 7
	width_  = depthCamera->ImageWidth();
	height_ = depthCamera->ImageHeight();
	depth_  = depthCamera->ImageDepth();
	format_ = depthCamera->ImageFormat();
#else
	width_      = depthCamera->GetImageWidth();
	height_     = depthCamera->GetImageHeight();
	depth_      = depthCamera->GetImageDepth();
	format_     = depthCamera->GetImageFormat();
#endif

	// newDepthFrameConnection = depthCamera->ConnectNewDepthFrame(
	//     boost::bind(&DepthCam::OnNewDepthFrame,
	//       this, _1, _2, _3, _4, _5));

	newRGBPointCloudConnection = depthCamera->ConnectNewRGBPointCloud(
	  boost::bind(&DepthCam::OnNewRGBPointCloud, this, _1, _2, _3, _4, _5));

	// newImageFrameConnection = depthCamera->ConnectNewImageFrame(
	//     boost::bind(&DepthCam::OnNewImageFrame,
	//       this, _1, _2, _3, _4, _5));

	parentSensor->SetActive(true);
}

/** on Gazebo reset
 */
void
DepthCam::Reset()
{
}

void
DepthCam::OnNewDepthFrame(const float *      _image,
                          unsigned int       _width,
                          unsigned int       _height,
                          unsigned int       _depth,
                          const std::string &_format)
{
	// printf("DepthCam: New Frame Depth\n");
	// printf("DepthCam: format: %s\n", _format.c_str());
}

/**
 * Callback with new point cloud RGBPOINTS is in the format x,y,z,uint_32 rgba
 */
void
DepthCam::OnNewRGBPointCloud(const float *      _pcd,
                             unsigned int       _width,
                             unsigned int       _height,
                             unsigned int       _depth,
                             const std::string &_format)
{
	// printf("DepthCam: New Frame RGB\n");
	// printf("DepthCam: format: %s\n", _format.c_str());

	//Construct point cloud message:
	msgs::PointCloud msg;
	//and fill with data
	for (unsigned int i = 0; i < _width * _height * 4; i = i + 4) {
		// printf("DepthCam: data: %f,%f,%f,%f\n", _pcd[i+0], _pcd[i+1], _pcd[i+2], _pcd[i+3]);
		msgs::Vector3d *point = msg.add_points();
		point->set_x(_pcd[i + 0]);
		point->set_y(_pcd[i + 1]);
		point->set_z(_pcd[i + 2]);
		//_pcd[i+4] would be the color
	}
	pcl_pub_->Publish(msg);
}

void
DepthCam::OnNewImageFrame(const unsigned char *_image,
                          unsigned int         _width,
                          unsigned int         _height,
                          unsigned int         _depth,
                          const std::string &  _format)
{
	// printf("DepthCam: New Frame Img\n");
	// printf("DepthCam: format: %s\n", _format.c_str());
}
