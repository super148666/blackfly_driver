/*  
    This code was developed by Chao Zhang for using with FLIR blackfly camera
*/

/*
    This file is part of blackfly_driver

    blackfly_driver is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    blackfly_driver is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with blackfly_driver.  If not, see <https://www.gnu.org/licenses/>.
*/

/*-*-C++-*-*/
/**
 * @file        blackfly_driver.h
 * @author      Chao Zhang
 * @created     June 21, 2018
 * @last edit   June 21, 2018
 * @brief       header of C++ Class warpper for blackfly ros driver
 * 
 * @attention   Copyright (C) 2018
*/
#ifndef FLYCAPTURE_DRIVER_H
#define FLYCAPTURE_DRIVER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <spinnaker/Spinnaker.h>
#include <spinnaker/SpinGenApi/SpinnakerGenApi.h>
#include <iostream>
#include <sstream>
#include "node_map_info.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;
using namespace cv;
using namespace image_transport;

class blackfly_driver {
    public:
        blackfly_driver(ros::NodeHandle nh);
        ~blackfly_driver();
        void configure();
        void spin(int frequency = -1);
        void spinOnce();
        void printCameraInfo();
        bool isOk();
    protected:
        void _readParams(ros::NodeHandle nh);
        bool _isSubscribed();
        vector<ImagePtr> _grabImages(vector<CameraPtr> pCams, PixelFormatEnums imgFormat = PixelFormat_Mono8, ColorProcessingAlgorithm algorithm = DEFAULT);
        vector<sensor_msgs::ImagePtr> _convertToImagePtr(vector<ImagePtr> pImages, string img_encoding = sensor_msgs::image_encodings::MONO8);
        void _publish(vector<sensor_msgs::ImagePtr> imgPtrs, vector<ros::Publisher> imgPubs);
        void _cameraInit();
        ros::NodeHandle _nh;
        int _numCameras;
        vector<string> _topicNames;
        vector<ros::Publisher> _vImgPubs;
        vector<sensor_msgs::ImagePtr> _vpRosImgs;
        vector<string> _vSerials;
        string _ros_img_encoding;
        int _spinRate;
};


#endif
