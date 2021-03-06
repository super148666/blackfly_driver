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
 * @file        blackfly_driver.cpp
 * @author      Chao Zhang
 * @created     June 21, 2018
 * @last edit   June 21, 2018
 * @brief       source of C++ Class warpper for blackfly ros driver
 * 
 * @attention   Copyright (C) 2018
*/

#include "flycapture_driver.h"


blackfly_driver::blackfly_driver(ros::NodeHandle nh): 
_nh(nh)
{  
}

blackfly_driver::~blackfly_driver() {
    this->_camList.Clear();
    this->_system->ReleaseInstance();
}

void blackfly_driver::configure() {
    // read parameters
    _readParams(this->_nh);
    
    // Validate image encodings
    switch (this->_imgFormat) {
        case PixelFormat_Mono8:
            this->_ros_img_encoding = sensor_msgs::image_encodings::MONO8;
            break;
        
        case PixelFormat_Mono16:
            this->_ros_img_encoding = sensor_msgs::image_encodings::MONO16;
            break;

        case PixelFormat_RGB8:
            this->_ros_img_encoding = sensor_msgs::image_encodings::RGB8;
            break;

        case PixelFormat_RGBa8:
            this->_ros_img_encoding = sensor_msgs::image_encodings::RGBA8;
            break;

        case PixelFormat_RGB16:
            this->_ros_img_encoding = sensor_msgs::image_encodings::RGB16;
            break;

        case PixelFormat_RGBa16:
            this->_ros_img_encoding = sensor_msgs::image_encodings::RGBA16;
            break;

        case PixelFormat_BGR8:
            this->_ros_img_encoding = sensor_msgs::image_encodings::BGR8;
            break;

        case PixelFormat_BGRa8:
            this->_ros_img_encoding = sensor_msgs::image_encodings::BGRA8;
            break;

        case PixelFormat_BGR16:
            this->_ros_img_encoding = sensor_msgs::image_encodings::BGR16;
            break;

        case PixelFormat_BGRa16:
            this->_ros_img_encoding = sensor_msgs::image_encodings::BGRA16;
            break;

        default:
            ROS_ERROR_STREAM("The desired format is not supported.");
            ROS_ERROR_STREAM("Supported format:\n" << 
                             "  PixelFormat_Mono8\n" << 
                             "  PixelFormat_Mono16\n" << 
                             "  PixelFormat_RGB8\n" << 
                             "  PixelFormat_RGBa8\n" << 
                             "  PixelFormat_RGB16\n" << 
                             "  PixelFormat_RGBa16\n" << 
                             "  PixelFormat_BGR8\n" << 
                             "  PixelFormat_BGRa8\n" << 
                             "  PixelFormat_BGR16\n" << 
                             "  PixelFormat_BGRa16\n" << 
                             "Aborting.");
            exit(-1);
    }
    ROS_INFO("Done image format check.");

    // create camera management instance
    this->_system = System::GetInstance();
	ROS_INFO("Done get system instance.");
	
    // get camera list
    this->_camList = this->_system->GetCameras();
	ROS_INFO("Done get cameras list.");
	
    // check num of cameras available
    if (this->_numCameras > this->_camList.GetSize())
    {
        ROS_ERROR("Not enough cameras available.");
        ROS_ERROR_STREAM("Available: " << this->_camList.GetSize());
        ROS_ERROR_STREAM("Required:  " << this->_numCameras);
        ROS_ERROR("Aborting.");
        exit(-1);
    }
	ROS_INFO("Done cameras num check.");
	
    // get camera instance by serial numbers
    for (int i = 0; i < this->_numCameras; i++) {
        if (this->_camList.GetBySerial(this->_vSerials[i])==NULL) {
            ROS_ERROR_STREAM("Unable to find camera with serial number: " << this->_vSerials[i] << ". Aborting.");
            exit(-1);
        }
        this->_vpCams.push_back(this->_camList.GetBySerial(this->_vSerials[i]));
        this->_vpCams[i]->Init();
    }
    ROS_INFO("Done get camera instance.");

    // init publishers
    for (int i = 0; i < this->_numCameras; i++) {
        ros::Publisher pub = this->_nh.advertise<sensor_msgs::Image>(this->_topicNames[i], 1);
        this->_vImgPubs.push_back(pub);
    }
    ROS_INFO_STREAM("Done init publisher. size: " << this->_vImgPubs.size());
    ROS_INFO("Done configure");

}

void blackfly_driver::spin(int frequency) {
    if (frequency <= 0) frequency = this->_spinRate;
    ros::Rate r(frequency);
    while(isOk()) {
        spinOnce();
        r.sleep();
    }
}

void blackfly_driver::spinOnce() {
    // grab images
    this->_vpImgs = _grabImages(this->_vpCams, this->_imgFormat, this->_algorithm);

    // convert raw images into ros images
    this->_vpRosImgs = _convertToImagePtr(this->_vpImgs, this->_ros_img_encoding);

    // publish images
    _publish(this->_vpRosImgs, this->_vImgPubs);
}

bool blackfly_driver::isOk() {
    return ros::ok();
}

void blackfly_driver::printCameraInfo() {
	//_cameraInit();
    // print info for all used cameras
    for (int i = 0; i < this->_numCameras; i++) {
        ROS_INFO_STREAM("Print info for " << i << " camera");
        RunSingleCamera(this->_vpCams[i]);
    }
}

void blackfly_driver::_readParams(ros::NodeHandle nh) {
    // std::string pkg_path = ros::package::getPath("blackfly_driver")+"/";
    string paramName;
    
    // read num of cameras
    paramName = "num_of_cameras";
    if(!nh.getParam(paramName, this->_numCameras)) {
        ROS_ERROR_STREAM("Please assign " << paramName << ".\n" <<  
                         "Aborting.");
        exit(-1);
    }
    
    // validate num of cameras
    if (this->_numCameras <= 0) {
        ROS_ERROR_STREAM("Invalid " << paramName << ": " << this->_numCameras << '\n' << 
                         "Aborting.");
        exit(-1);
    }
    ROS_INFO_STREAM(paramName << ": " << this->_numCameras << ".");

    
    // read topic names
    paramName = "topic";
    this->_topicNames.clear();
    for (int i = 1; i <= this->_numCameras; i++) {
        string paramValue;
        if(!nh.getParam(paramName + to_string(i), paramValue)) {
            ROS_ERROR_STREAM("Please assign " << paramName + to_string(i) << ".\n" << 
                             "Aborting.");
            exit(-1);
        }
        this->_topicNames.push_back(paramValue);
        ROS_INFO_STREAM(paramName << to_string(i) << ": " << paramValue << ".");
    }
    


    // read serial numbers of cameras
    paramName = "serial";
    this->_vSerials.clear();
    for (int i = 1; i <= this->_numCameras; i++) {
        string paramValue;
        if(!nh.getParam(paramName + to_string(i), paramValue)) {
            ROS_ERROR_STREAM("Please assign " << paramName + to_string(i) << ".\n" << 
                             "Aborting.");
            exit(-1);
        }
        this->_vSerials.push_back(paramValue);
        ROS_INFO_STREAM(paramName << to_string(i) << ": " << paramValue << ".");
    }

    // read pixel format for cameras
    paramName = "format";
    string strFormat = "";
    if(!nh.getParam(paramName, strFormat)) {
        ROS_ERROR_STREAM("Please assign " << paramName << ".\n" <<  
                         "Aborting.");
        exit(-1);
    }
    ROS_INFO_STREAM(paramName << ": " << strFormat << ".");
    if (strFormat == "mono8") this->_imgFormat = PixelFormat_Mono8;
    else if(strFormat == "mono16") this->_imgFormat = PixelFormat_Mono16;
    else if(strFormat == "rgb8") this->_imgFormat = PixelFormat_RGB8;
    else if(strFormat == "rgba8") this->_imgFormat = PixelFormat_RGBa8;
    else if(strFormat == "rgb16") this->_imgFormat = PixelFormat_RGB16;
    else if(strFormat == "rgba16") this->_imgFormat = PixelFormat_RGBa16;
    else if(strFormat == "bgr8") this->_imgFormat = PixelFormat_BGR8;
    else if(strFormat == "bgra8") this->_imgFormat = PixelFormat_BGRa8;
    else if(strFormat == "bgr16") this->_imgFormat = PixelFormat_BGR16;
    else if(strFormat == "bgra16") this->_imgFormat = PixelFormat_BGRa16;
    else {
		ROS_ERROR_STREAM("The desired format is not supported.");
		ROS_ERROR_STREAM("Supported format:\n" << 
						 "  mono8\n" << 
						 "  mono16\n" << 
						 "  rgb8\n" << 
						 "  rgba8\n" << 
						 "  rgb16\n" << 
						 "  rgba16\n" << 
						 "  bgr8\n" << 
						 "  bgra8\n" << 
						 "  bgr16\n" << 
						 "  bgra16\n" << 
						 "Aborting.");
		exit(-1);
	}
    
		

    // read color processing algorithm for cameras
    paramName = "algorithm";
    string strAlogrithm = "";
    if(!nh.getParam(paramName, strAlogrithm)) {
        ROS_ERROR_STREAM("Please assign " << paramName << ".\n" <<  
                         "Aborting.");
        exit(-1);
    }
    ROS_INFO_STREAM(paramName << ": " << strAlogrithm << ".");
    if (strAlogrithm == "DEFAULT") this->_algorithm = DEFAULT;
    else if(strAlogrithm == "NO_COLOR_PROCESSING") this->_algorithm = NO_COLOR_PROCESSING;
    else if(strAlogrithm == "NEAREST_NEIGHBOR") this->_algorithm = NEAREST_NEIGHBOR;
    else if(strAlogrithm == "EDGE_SENSING") this->_algorithm = EDGE_SENSING;
    else if(strAlogrithm == "HQ_LINEAR") this->_algorithm = HQ_LINEAR;
    else if(strAlogrithm == "RIGOROUS") this->_algorithm = RIGOROUS;
    else if(strAlogrithm == "IPP") this->_algorithm = IPP;
    else if(strAlogrithm == "DIRECTIONAL_FILTER") this->_algorithm = DIRECTIONAL_FILTER;
    else {
		ROS_ERROR_STREAM("The desired format is not supported.");
		ROS_ERROR_STREAM("Supported format:\n" << 
						 "  mono8\n" << 
						 "  mono16\n" << 
						 "  rgb8\n" << 
						 "  rgba8\n" << 
						 "  rgb16\n" << 
						 "  rgba16\n" << 
						 "  bgr8\n" << 
						 "  bgra8\n" << 
						 "  bgr16\n" << 
						 "  bgra16\n" << 
						 "Aborting.");
		exit(-1);
	}

    // read spin rate for the publisher
    paramName = "spin_rate";
    if(!nh.getParam(paramName, this->_spinRate)) {
        ROS_ERROR_STREAM("Please assign " << paramName << ".\n" <<  
                         "Aborting.");
        exit(-1);
    }
    ROS_INFO_STREAM(paramName << ": " << this->_spinRate << ".");

    ROS_INFO("Done read parameters.");
}

vector<ImagePtr> blackfly_driver::_grabImages(vector<CameraPtr> pCams, PixelFormatEnums imgFormat, ColorProcessingAlgorithm algorithm){
    //_cameraInit();
    vector<ImagePtr> result;
    CameraPtr pCam = NULL;
    ROS_DEBUG("*** IMAGE ACQUISITION ***");

    // Check empty vector
    if (pCams.empty()) {
        ROS_ERROR("No cameras available for acquisition. Aborting.");
        exit(-1);
    }
    try
    {
        // Prepare each camera to acquire images
        for (int i = 0; i < pCams.size(); i++)
        {
            // Select camera
            pCam = pCams[i];
            // Set acquisition mode to continuous
            
            CEnumerationPtr ptrAcquisitionMode = pCam->GetNodeMap().GetNode("AcquisitionMode");
            if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
            {
                ROS_ERROR_STREAM("Unable to set acquisition mode to continuous (node retrieval; camera " << i << "). Aborting.");
                exit(-1);
            }
            
            CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
            if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
            {
                ROS_ERROR_STREAM("Unable to set acquisition mode to continuous (entry 'continuous' retrieval " << i << "). Aborting.");
                exit(-1);
            }
            int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
            ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);
            ROS_DEBUG_STREAM("Camera " << i << " acquisition mode set to continuous.");
            
            /*
            CEnumEntryPtr ptrAcquisitionModeSingleFrame = ptrAcquisitionMode->GetEntryByName("SingleFrame");
            if (!IsAvailable(ptrAcquisitionModeSingleFrame) || !IsReadable(ptrAcquisitionModeSingleFrame))
            {
                ROS_ERROR_STREAM("Unable to set acquisition mode to singleframe (entry 'singleframe' retrieval " << i << "). Aborting.");
                exit(-1);
            }
            int64_t acquisitionModeSingleFrame = ptrAcquisitionModeSingleFrame->GetValue();
            ptrAcquisitionMode->SetIntValue(acquisitionModeSingleFrame);
            ROS_DEBUG_STREAM("Camera " << i << " acquisition mode set to singleframe.");
            */
            // Begin acquiring images
            pCam->BeginAcquisition();
            ROS_DEBUG_STREAM("Camera " << i << " started acquiring images.");
        }
        
        // Retrieve, convert, and save images for each camera
        for (int i = 0; i < pCams.size(); i++)
        {
            try
            {
                // Select camera
                pCam = pCams[i];
                // Retrieve next received image and ensure image completion
                ImagePtr pResultImage = pCam->GetNextImage();
                if (pResultImage->IsIncomplete())
                {
                    ROS_WARN_STREAM("Image incomplete with image status " << pResultImage->GetImageStatus() << ".");
                }
                else
                {
                    // Print image information
					ROS_DEBUG_STREAM("Camera " << i << " grabbed image , width = " << pResultImage->GetWidth() << ", height = " << pResultImage->GetHeight() << '.');
					// Convert image to desired format
					ImagePtr convertedImage = pResultImage->Convert(imgFormat, algorithm);
					result.push_back(convertedImage);
                }
                
                // Release image
                pResultImage->Release();
            }
            catch (Spinnaker::Exception &e)
            {
                ROS_ERROR_STREAM("Error: " << e.what() << '.');
                exit(-1);
            }
        }
        
        // End acquisition for each camera
        for (int i = 0; i < pCams.size(); i++)
        {
            // End acquisition
            pCams[i]->EndAcquisition();
        }
    }
    catch (Spinnaker::Exception &e)
    {
        ROS_ERROR_STREAM("Error: " << e.what() << '.');
        exit(-1);
    }
    return result;
}

vector<sensor_msgs::ImagePtr> blackfly_driver::_convertToImagePtr(vector<ImagePtr> pImages, string img_encoding) {
    
    vector<sensor_msgs::ImagePtr> ros_imgs;
    for (int i = 0; i < pImages.size(); i++) {
        // Consider padding
        unsigned int XPadding = pImages[i]->GetXPadding();
        unsigned int YPadding = pImages[i]->GetYPadding();
        unsigned int rowsize = pImages[i]->GetWidth();
        unsigned int colsize = pImages[i]->GetHeight();

        // Convert raw data into cv::Mat
        Mat cvimg = cv::Mat(colsize + YPadding, rowsize + XPadding, CV_8UC3, pImages[i]->GetData(), pImages[i]->GetStride());
        
        // Create ros image pointer
        sensor_msgs::ImagePtr ros_img = cv_bridge::CvImage(std_msgs::Header(), img_encoding, cvimg).toImageMsg();
        ros_imgs.push_back(ros_img);
    }
	return ros_imgs;
}

void blackfly_driver::_publish(vector<sensor_msgs::ImagePtr> imgPtrs, vector<ros::Publisher> imgPubs) {
    // static time variable
    static const ros::Duration fixed_interval(10);
    static ros::Time start_active = ros::Time::now();
    int totalSubscribers = 0;

    // Size of image and publisher must match and be greater than zero.
    if (imgPtrs.size()==0 || imgPubs.size()==0 || imgPubs.size()!=imgPtrs.size()) {
        ROS_ERROR("Invalid size of images or publisher.");
        ROS_ERROR_STREAM("size of images:     " << imgPtrs.size() << 
                         "size of publishers: " << imgPubs.size() << 
                         "Aborting.");
        //exit(-1);
        return;
    }

    // Check activation
    static bool active = false;
    if (active) {
        if (ros::Time::now() - start_active > fixed_interval) {
            // Collect total number of subscribers
            for (int i = 0; i < this->_numCameras; i++) {
                totalSubscribers += imgPubs[i].getNumSubscribers();
            }
            if (totalSubscribers == 0) {
                ROS_INFO("Publisher deactivated.");
                active = false;
            }
        }
    }
    else {
        // Collect total number of subscribers
        for (int i = 0; i < this->_numCameras; i++) {
            totalSubscribers += imgPubs[i].getNumSubscribers();
        }
        if (totalSubscribers > 0) {
            ROS_INFO("Publisher activated.");
            active = true;
            start_active = ros::Time::now();
        }
    }

    // Publish
    if (active) {
        for (int i = 0; i < this->_numCameras; i++) {
            imgPubs[i].publish(*imgPtrs[i]);
        }
    }
}

void blackfly_driver::_cameraInit() {
	for (int i = 0; i < this->_numCameras; i++) {
		this->_vpCams[i]->Init();
	}
}
