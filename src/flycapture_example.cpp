#include "FlyCapture2.h"
#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace FlyCapture2;
using namespace std;
using namespace cv;

int numCameras;
vector<string> topicNames;
vector<int> vSerials;
PixelFormat imgFormat;
int spinRate;
vector<ros::Publisher> vImgPubs;
vector<sensor_msgs::ImagePtr> vpRosImgs;
string ros_img_encoding;

int packetSize = 8100;
int packetDelay = 6250;
Mode mode = MODE_1;
bool packetResend = true;
int offsetX = 0;
int offsetY = 0;
int width = 644;
int height = 482;
bool calibrate = false;
string config_filepath;

Mat Kl,Kr,Dl,Dr,Rl,Rr,Pl,Pr;

void PrintError(FlyCapture2::Error error) { error.PrintErrorTrace(); }

void PrintBuildInfo()
{
    FC2Version fc2Version;
    Utilities::GetLibraryVersion(&fc2Version);

    ostringstream version;
    version << "FlyCapture2 library version: " << fc2Version.major << "."
            << fc2Version.minor << "." << fc2Version.type << "."
            << fc2Version.build;
    cout << version.str() << endl;

    ostringstream timeStamp;
    timeStamp << "Application build date: " << __DATE__ << " " << __TIME__;
    cout << timeStamp.str() << endl << endl;
}

void PrintCameraInfo(CameraInfo *pCamInfo)
{
    ostringstream macAddress;
    macAddress << hex << setw(2) << setfill('0')
               << (unsigned int)pCamInfo->macAddress.octets[0] << ":" << hex
               << setw(2) << setfill('0')
               << (unsigned int)pCamInfo->macAddress.octets[1] << ":" << hex
               << setw(2) << setfill('0')
               << (unsigned int)pCamInfo->macAddress.octets[2] << ":" << hex
               << setw(2) << setfill('0')
               << (unsigned int)pCamInfo->macAddress.octets[3] << ":" << hex
               << setw(2) << setfill('0')
               << (unsigned int)pCamInfo->macAddress.octets[4] << ":" << hex
               << setw(2) << setfill('0')
               << (unsigned int)pCamInfo->macAddress.octets[5];

    ostringstream ipAddress;
    ipAddress << (unsigned int)pCamInfo->ipAddress.octets[0] << "."
              << (unsigned int)pCamInfo->ipAddress.octets[1] << "."
              << (unsigned int)pCamInfo->ipAddress.octets[2] << "."
              << (unsigned int)pCamInfo->ipAddress.octets[3];

    ostringstream subnetMask;
    subnetMask << (unsigned int)pCamInfo->subnetMask.octets[0] << "."
               << (unsigned int)pCamInfo->subnetMask.octets[1] << "."
               << (unsigned int)pCamInfo->subnetMask.octets[2] << "."
               << (unsigned int)pCamInfo->subnetMask.octets[3];

    ostringstream defaultGateway;
    defaultGateway << (unsigned int)pCamInfo->defaultGateway.octets[0] << "."
                   << (unsigned int)pCamInfo->defaultGateway.octets[1] << "."
                   << (unsigned int)pCamInfo->defaultGateway.octets[2] << "."
                   << (unsigned int)pCamInfo->defaultGateway.octets[3];

    cout << endl;
    cout << "*** CAMERA INFORMATION ***" << endl;
    cout << "Serial number - " << pCamInfo->serialNumber << endl;
    cout << "Camera model - " << pCamInfo->modelName << endl;
    cout << "Camera vendor - " << pCamInfo->vendorName << endl;
    cout << "Sensor - " << pCamInfo->sensorInfo << endl;
    cout << "Resolution - " << pCamInfo->sensorResolution << endl;
    cout << "Firmware version - " << pCamInfo->firmwareVersion << endl;
    cout << "Firmware build time - " << pCamInfo->firmwareBuildTime << endl;
    cout << "GigE version - " << pCamInfo->gigEMajorVersion << "."
         << pCamInfo->gigEMinorVersion << endl;
    cout << "User defined name - " << pCamInfo->userDefinedName << endl;
    cout << "XML URL 1 - " << pCamInfo->xmlURL1 << endl;
    cout << "XML URL 2 - " << pCamInfo->xmlURL2 << endl;
    cout << "MAC address - " << macAddress.str() << endl;
    cout << "IP address - " << ipAddress.str() << endl;
    cout << "Subnet mask - " << subnetMask.str() << endl;
    cout << "Default gateway - " << defaultGateway.str() << endl << endl;
}

void PrintStreamChannelInfo(GigEStreamChannel *pStreamChannel)
{
    // char ipAddress[32];
    ostringstream ipAddress;
    ipAddress << (unsigned int)pStreamChannel->destinationIpAddress.octets[0]
              << "."
              << (unsigned int)pStreamChannel->destinationIpAddress.octets[1]
              << "."
              << (unsigned int)pStreamChannel->destinationIpAddress.octets[2]
              << "."
              << (unsigned int)pStreamChannel->destinationIpAddress.octets[3];

    cout << "Network interface - " << pStreamChannel->networkInterfaceIndex
         << endl;
    cout << "Host Port - " << pStreamChannel->hostPort << endl;
    cout << "Do not fragment bit - "
         << (pStreamChannel->doNotFragment ? "Enabled" : "Disabled") << endl;
    cout << "Packet size - " << pStreamChannel->packetSize << endl;
    cout << "Inter packet delay - " << pStreamChannel->interPacketDelay << endl;
    cout << "Destination IP address - " << ipAddress.str() << endl;
    cout << "Source port (on camera) - " << pStreamChannel->sourcePort << endl
         << endl;
}

int DisableHeartbeat(GigECamera& cam)
{
    const unsigned int k_GVCPCapabilityAddr = 0x0934;
    const unsigned int k_GVCPConfigAddr = 0x0954;
    unsigned int regVal;

    // Determine if heartbeat can be disabled by reading the GVCP Capability register
    FlyCapture2::Error error = cam.ReadGVCPRegister(k_GVCPCapabilityAddr, &regVal);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    const unsigned BitMask = 0x20000000;
    const bool CanDisableHeartbeat = ((regVal & BitMask) == BitMask);

    if (CanDisableHeartbeat)
    {
        error = cam.ReadGVCPRegister(k_GVCPConfigAddr, &regVal);
        if (error != PGRERROR_OK)
        {
            PrintError(error);
            return -1;
        }

        // Disable heartbeat by setting GVCP Configuration register's bit 31 to 1
        regVal |= 0x00000001; 

        error = cam.WriteGVCPRegister(k_GVCPConfigAddr, regVal);
        if (error != PGRERROR_OK)
        {
            PrintError(error);
            return -1;
        }

        cout << endl;
        cout << "NOTE: GigE camera's heartbeat is disabled in Debug Mode" << endl;
        cout << "      Please power cycle the camera to re-enable the heartbeat." << endl;
    }

    return 0;
}

void ReadParams(ros::NodeHandle nh) {
    std::string pkg_path = ros::package::getPath("blackfly_driver")+"/";
    string paramName;
    
    // read num of cameras
    paramName = "num_of_cameras";
    if(!nh.getParam(paramName, numCameras)) {
        ROS_ERROR_STREAM("Please assign " << paramName << ".\n" <<  
                         "Aborting.");
        exit(-1);
    }
    
    // validate num of cameras
    if (numCameras <= 0) {
        ROS_ERROR_STREAM("Invalid " << paramName << ": " << numCameras << '\n' << 
                         "Aborting.");
        exit(-1);
    }
    ROS_INFO_STREAM(paramName << ": " << numCameras << ".");

    
    // read topic names
    paramName = "topic";
    topicNames.clear();
    for (int i = 1; i <= numCameras; i++) {
        string paramValue;
        if(!nh.getParam(paramName + to_string(i), paramValue)) {
            ROS_ERROR_STREAM("Please assign " << paramName + to_string(i) << ".\n" << 
                             "Aborting.");
            exit(-1);
        }
        topicNames.push_back(paramValue);
        ROS_INFO_STREAM(paramName << to_string(i) << ": " << paramValue << ".");
    }
    


    // read serial numbers of cameras
    paramName = "serial";
    vSerials.clear();
    for (int i = 1; i <= numCameras; i++) {
        string paramValue;
        if(!nh.getParam(paramName + to_string(i), paramValue)) {
            ROS_ERROR_STREAM("Please assign " << paramName + to_string(i) << ".\n" << 
                             "Aborting.");
            exit(-1);
        }
        vSerials.push_back(stoi(paramValue));
        ROS_INFO_STREAM(paramName << to_string(i) << ": " << paramValue << ".");
    }

    // read pixel format for cameras
    paramName = "format";
    string strFormat;
    if(!nh.getParam(paramName, strFormat)) {
        ROS_ERROR_STREAM("Please assign " << paramName << ".\n" <<  
                         "Aborting.");
        exit(-1);
    }
    ROS_INFO_STREAM(paramName << ": " << strFormat << ".");
    if (strFormat == "mono8") imgFormat = PIXEL_FORMAT_MONO8;
    else if(strFormat == "rgb8") imgFormat = PIXEL_FORMAT_RGB;
    else if(strFormat == "bgr8") imgFormat = PIXEL_FORMAT_BGR;
    else {
		ROS_ERROR_STREAM("The desired format is not supported.");
		ROS_ERROR_STREAM("Supported format:\n" << 
						 "  mono8\n" <<
						 "  rgb8\n" <<
						 "  bgr8\n" <<
						 "Aborting.");
		exit(-1);
	}

    // read spin rate for the publisher
    paramName = "spin_rate";
    if(!nh.getParam(paramName, spinRate)) {
        ROS_ERROR_STREAM("Please assign " << paramName << ".\n" <<  
                         "Aborting.");
        exit(-1);
    }
    ROS_INFO_STREAM(paramName << ": " << spinRate << ".");
	
	int* int_ptr;
	int int_reading;
	// read packet_size
    paramName = "packet_size";
    int_ptr = &packetSize;
    if(!nh.getParam(paramName, int_reading)) {
        ROS_WARN_STREAM("Cannot find " << paramName << ".\n" <<  
                         "Default value will be used.");
    }
    else *int_ptr = int_reading;
    ROS_INFO_STREAM(paramName << ": " << *int_ptr << ".");
    
    // read packet_delay
    paramName = "packet_delay";
    int_ptr = &packetDelay;
    if(!nh.getParam(paramName, int_reading)) {
        ROS_WARN_STREAM("Cannot find " << paramName << ".\n" <<  
                         "Default value will be used.");
    }
    else *int_ptr = int_reading;
    ROS_INFO_STREAM(paramName << ": " << *int_ptr << ".");
    
    // read offsetX
    paramName = "offsetX";
    int_ptr = &offsetX;
    if(!nh.getParam(paramName, int_reading)) {
        ROS_WARN_STREAM("Cannot find " << paramName << ".\n" <<  
                         "Default value will be used.");
    }
    else *int_ptr = int_reading;
    ROS_INFO_STREAM(paramName << ": " << *int_ptr << ".");
    
    // read offsetY
    paramName = "offsetY";
    int_ptr = &offsetY;
    if(!nh.getParam(paramName, int_reading)) {
        ROS_WARN_STREAM("Cannot find " << paramName << ".\n" <<  
                         "Default value will be used.");
    }
    else *int_ptr = int_reading;
    ROS_INFO_STREAM(paramName << ": " << *int_ptr << ".");
    
    // read width
    paramName = "width";
    int_ptr = &width;
    if(!nh.getParam(paramName, int_reading)) {
        ROS_WARN_STREAM("Cannot find " << paramName << ".\n" <<  
                         "Default value will be used.");
    }
    else *int_ptr = int_reading;
    ROS_INFO_STREAM(paramName << ": " << *int_ptr << ".");
    
    // read height
    paramName = "height";
    int_ptr = &height;
    if(!nh.getParam(paramName, int_reading)) {
        ROS_WARN_STREAM("Cannot find " << paramName << ".\n" <<  
                         "Default value will be used.");
    }
    else *int_ptr = int_reading;
    ROS_INFO_STREAM(paramName << ": " << *int_ptr << ".");
    
    // read image_mode
    paramName = "image_mode";
    if(!nh.getParam(paramName, int_reading)) {
        ROS_WARN_STREAM("Cannot find " << paramName << ".\n" <<  
                         "Default value will be used.");
    }
    else mode = (Mode)int_reading;
    ROS_INFO_STREAM(paramName << ": " << mode << ".");
    
    string param_name;
	int param_int;
	int* param_int_ptr;
	float param_float;
	float* param_float_ptr;
	string param_string;
	string* param_string_ptr;
	bool param_bool;
	bool* param_bool_ptr;
	bool error_exit = false;
	
    param_name = "/calibrate";
    param_bool_ptr = &calibrate;
    if (!nh.getParam(param_name, param_bool)) {
		ROS_WARN_STREAM("Missing parameters " << param_name << ".");
		ROS_WARN_STREAM("Default setting will be used for " << param_name);
	} else *param_bool_ptr = param_bool;
	ROS_INFO_STREAM("Setting " << param_name << " to " << ((*param_bool_ptr)?"true":"false") << ".");
	
	if (calibrate) {
		if (numCameras != 2) {
			ROS_ERROR("calibration must be used with 2 cameras.");
			ROS_ERROR("Aborting.");
			exit(-1);
		}
		// load save_path
		param_name = "/file_path";
		param_string_ptr = &config_filepath;
		if (!nh.getParam(param_name, param_string)) {
			ROS_WARN_STREAM("Missing parameters " << param_name << ".");
			ROS_WARN_STREAM("Default setting will be used for " << param_name);
		} else *param_string_ptr = param_string;
		if (param_string_ptr->empty()) {
			ROS_ERROR_STREAM("Invalid empty " << param_name << ".");
			error_exit = true;
		}
		ROS_INFO_STREAM("Setting " << param_name << " to " << *param_string_ptr << ".");
		
		
		bool absolute_path = false;
		param_name = "absolute_path";
		param_bool_ptr = &absolute_path;
		if (!nh.getParam(param_name, param_bool)) {
			ROS_WARN_STREAM("Missing parameters " << param_name << ".");
			ROS_WARN_STREAM("Default setting will be used for " << param_name);
		} else *param_bool_ptr = param_bool;
		ROS_INFO_STREAM("Setting " << param_name << " to " << ((*param_bool_ptr)?"true":"false") << ".");
		
		if(!absolute_path) {
			config_filepath.insert(0,pkg_path);
		}
	}
    ROS_INFO("Done read parameters.");
}

void LoadCalibrationMatrix(string filename) {
	ROS_INFO("Start load calibration matrixs.");
	FileStorage fs(filename, FileStorage::READ);
	fs["Kl"] >> Kl;
	fs["Kr"] >> Kr;
	fs["Dl"] >> Dl;
	fs["Dr"] >> Dr;
	fs["Rl"] >> Rl;
	fs["Rr"] >> Rr;
	fs["Pl"] >> Pl;
	fs["Pr"] >> Pr;
	fs.release();
	
	ROS_INFO("Done load calibration matrixs.");
}


int main(int argc, char ** argv)
{
	ros::init(argc, argv, "flycapture_driver");
	ros::NodeHandle nh;
	ReadParams(nh);
	
	LoadCalibrationMatrix(config_filepath);
	
	
	Mat map_1[2], map_2[2];
	initUndistortRectifyMap(Kl, Dl, Rl, Pl, Size(width, height),  CV_16SC2, map_1[0], map_2[0]);
	initUndistortRectifyMap(Kr, Dr, Rr, Pr, Size(width, height),  CV_16SC2, map_1[1], map_2[1]);
	
	// Validate image encodings
    switch (imgFormat) {
        case PIXEL_FORMAT_MONO8:
            ros_img_encoding = sensor_msgs::image_encodings::MONO8;
            break;
            
        case PIXEL_FORMAT_RGB:
            ros_img_encoding = sensor_msgs::image_encodings::RGB8;
            break;

        case PIXEL_FORMAT_BGR:
            ros_img_encoding = sensor_msgs::image_encodings::BGR8;
            break;

        default:
            ROS_ERROR_STREAM("The desired format is not supported.");
            ROS_ERROR_STREAM("Supported format:\n" << 
                             "  PixelFormat_Mono8\n" <<
                             "  PixelFormat_RGB8\n" <<
                             "  PixelFormat_BGR8\n" <<
                             "Aborting.");
            exit(-1);
    }
	ROS_INFO("Done image format check.");
	
	// init publishers
    for (int i = 0; i < numCameras; i++) {
        ros::Publisher pub = nh.advertise<sensor_msgs::Image>(topicNames[i], 1);
        vImgPubs.push_back(pub);
    }
    ROS_INFO_STREAM("Done init publisher. size: " << vImgPubs.size());
	
    PrintBuildInfo();

    FlyCapture2::Error error;

    //
    // Initialize BusManager and retrieve number of cameras detected
    //
    BusManager busMgr;
    unsigned int numAllCameras;
    error = busMgr.GetNumOfCameras(&numAllCameras);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    ROS_INFO_STREAM("Number of cameras detected: " << numAllCameras << ".");

    //
    // Check to make sure at least two cameras are connected before
    // running example
    //
    if (numAllCameras < numCameras)
    {
        ROS_ERROR_STREAM("Insufficient number of cameras.\n" <<
						 "Make sure at least " << numCameras << " cameras are connected for example to run.");
        return -1;
    }

    //
    // Initialize an array of cameras
    //
    // *** NOTES ***
    // The size of the array is equal to the number of cameras detected.
    // The array of cameras will be used for connecting, configuring,
    // and capturing images.
    //
    GigECamera *pCameras = new GigECamera[numCameras];
    

    //
    // Prepare each camera to acquire images
    //
    // *** NOTES ***
    // For pseudo-simultaneous streaming, each camera is prepared as if it
    // were just one, but in a loop. Notice that cameras are selected with
    // an index. We demonstrate pseduo-simultaneous streaming because true
    // simultaneous streaming would require multiple process or threads,
    // which is too complex for an example.
    //
    for (unsigned int i = 0; i < numCameras; i++)
    {
        PGRGuid guid;
        error = busMgr.GetCameraFromSerialNumber(vSerials[i], &guid);
        if (error != PGRERROR_OK)
        {
            PrintError(error);
            delete[] pCameras;
            return -1;
        }

        // Connect to a camera
        error = pCameras[i].Connect(&guid);
        if (error != PGRERROR_OK)
        {
            PrintError(error);
            delete[] pCameras;
            return -1;
        }
        
        // Get the camera information
        CameraInfo camInfo;
        error = pCameras[i].GetCameraInfo(&camInfo);
        if (error != PGRERROR_OK)
        {
            PrintError(error);
            delete[] pCameras;
            return -1;
        }
        PrintCameraInfo(&camInfo);
        
        // enable packet resend
        GigEConfig config;
		error = pCameras[i].GetGigEConfig(&config);
		if (error != PGRERROR_OK)
        {
            PrintError(error);
            delete[] pCameras;
            return -1;
        }
		config.enablePacketResend = true;
		error = pCameras[i].SetGigEConfig(&config);
		if (error != PGRERROR_OK)
        {
            PrintError(error);
            delete[] pCameras;
            return -1;
        }
		ROS_INFO("Packet resend is enabled.");
		
		// set camera imaging mode
		bool supported = false;
		error = pCameras[i].QueryGigEImagingMode(mode, &supported);
		if (error != PGRERROR_OK)
        {
            PrintError(error);
            delete[] pCameras;
            return -1;
        }
		if (!supported) {
			ostringstream ostr_mode;
			ostr_mode << "The imaging mode " << mode << " is not supported.\n" << "Aborting.\n";
			ROS_ERROR_STREAM(ostr_mode.str());
			delete[] pCameras;
			return -1;
		}
		error = pCameras[i].SetGigEImagingMode(mode);
		if (error != PGRERROR_OK)
        {
            PrintError(error);
            delete[] pCameras;
            return -1;
        }
        ostringstream ostr_mode;
		ostr_mode << "The imaging mode is set to " << mode << ".\n";
        ROS_INFO_STREAM(ostr_mode.str());
		
		// set camera image setting
		GigEImageSettings settings;
		error = pCameras[i].GetGigEImageSettings(&settings);
		if (error != PGRERROR_OK)
        {
            PrintError(error);
            delete[] pCameras;
            return -1;
        }
		settings.offsetX = 0;
		settings.offsetY = 0;
		settings.width = 644;
		settings.height = 482;
		settings.pixelFormat = PIXEL_FORMAT_RGB8;
		error = pCameras[i].SetGigEImageSettings(&settings);
		if (error != PGRERROR_OK)
        {
            PrintError(error);
            delete[] pCameras;
            return -1;
        }
        ostringstream ostr_settings;
		ostr_settings << "Image setting:\n" <<
						"    offsetX: 		" << settings.offsetX << ".\n" <<
						"    offsetY: 		" << settings.offsetY << ".\n" <<
						"    width: 		" << settings.width << ".\n" <<
						"    height: 		" << settings.height << ".\n" <<
						"    pixelFormat: 	" << settings.pixelFormat << ".\n";
        ROS_INFO_STREAM(ostr_settings.str());
        
        // set packet size
		GigEProperty ps_gp;
		ps_gp.propType = PACKET_SIZE;
		error = pCameras[i].GetGigEProperty(&ps_gp);
		if (error != PGRERROR_OK)
        {
            PrintError(error);
            delete[] pCameras;
            return -1;
        }
		unsigned int temp_usint;
		error = pCameras[i].DiscoverGigEPacketSize(&temp_usint);
		if (error != PGRERROR_OK)
        {
            PrintError(error);
            delete[] pCameras;
            return -1;
        }
		ps_gp.value = 8100;
		error = pCameras[i].SetGigEProperty(&ps_gp);
		if (error != PGRERROR_OK)
        {
            PrintError(error);
            delete[] pCameras;
            return -1;
        }
		ROS_INFO_STREAM("The packet size for camera " << i << " is set to " << ps_gp.value << ".");
		
		// set packet delay
		GigEProperty pd_gp;
		ps_gp.propType = PACKET_DELAY;
		error = pCameras[i].GetGigEProperty(&pd_gp);
		if (error != PGRERROR_OK)
        {
            PrintError(error);
            delete[] pCameras;
            return -1;
        }
		pd_gp.value = pd_gp.max;
		error = pCameras[i].SetGigEProperty(&ps_gp);
		if (error != PGRERROR_OK)
        {
            PrintError(error);
            delete[] pCameras;
            return -1;
        }
		ROS_INFO_STREAM("The packet delay for camera " << i << " is set to " << pd_gp.value << ".");

		FC2Config fc2config;
		error = pCameras[i].GetConfiguration(&fc2config);
		fc2config.isochBusSpeed = BUSSPEED_S_FASTEST;
		fc2config.asyncBusSpeed = BUSSPEED_S_FASTEST;
		error = pCameras[i].SetConfiguration(&fc2config);
		ostringstream ostr_fc2config;
		ostr_fc2config << 
						"FC2Config:\n" <<
						"    numBuffers:" << fc2config.numBuffers << ".\n" <<
						"    numImageNotifications:" << fc2config.numImageNotifications << ".\n" <<
						"    minNumImageNotifications:" << fc2config.minNumImageNotifications << ".\n" <<
						"    grabTimeout:" << fc2config.grabTimeout << ".\n" <<
						"    grabMode:" << fc2config.grabMode << ".\n" <<
						"    highPerformanceRetrieveBuffer:" << fc2config.highPerformanceRetrieveBuffer << ".\n" <<
						"    isochBusSpeed:" << fc2config.isochBusSpeed << ".\n" <<
						"    asyncBusSpeed:" << fc2config.asyncBusSpeed << ".\n" <<
						"    bandwidthAllocation:" << fc2config.bandwidthAllocation << ".\n" <<
						"    registerTimeoutRetries:" << fc2config.registerTimeoutRetries << ".\n" <<
						"    registerTimeout:" << fc2config.registerTimeout << ".\n";
		ROS_INFO_STREAM(ostr_fc2config.str());
        

        // Turn trigger mode off
        TriggerMode trigMode;
        trigMode.onOff = false;
        error = pCameras[i].SetTriggerMode(&trigMode);
        if (error != PGRERROR_OK)
        {
            PrintError(error);
            delete[] pCameras;
            return -1;
        }

        // Start streaming on camera
        error = pCameras[i].StartCapture();
        if (error != PGRERROR_OK)
        {
            PrintError(error);
            delete[] pCameras;
            return -1;
        }
    }

    //
    // Retrieve images from all cameras
    //
    // *** NOTES ***
    // In order to work with simultaneous camera streams, nested loops are
    // needed. It is important that the inner loop be the one iterating
    // through the cameras; otherwise, all images will be grabbed from a
    // single camera before grabbing any images from another.
    //
    ros::Rate r(spinRate);
    int j = 0;
    bool success = true;
    while (ros::ok()) {
		j++;
		vpRosImgs.clear();
        for (unsigned int i = 0; i < numCameras; i++)
        {
            Image image, convertedImage;
            error = pCameras[i].RetrieveBuffer(&image);
            if (error != PGRERROR_OK)
            {
				success = false;
                PrintError(error);
                continue;
            }

            ROS_INFO_STREAM("get image from camera " << i << ".");
			error = image.Convert(imgFormat, &convertedImage);
			if (error != PGRERROR_OK)
			{
				PrintError(error);
				return -1;
			}
			unsigned int rowBytes = (double)convertedImage.GetReceivedDataSize()/(double)convertedImage.GetRows();
			cv::Mat cvimg = cv::Mat(convertedImage.GetRows(), convertedImage.GetCols(), CV_8UC3, convertedImage.GetData(),rowBytes);
			if (calibrate) {
				remap(cvimg, cvimg, map_1[i], map_2[i], INTER_LINEAR);
			}
			sensor_msgs::ImagePtr ros_img = cv_bridge::CvImage(std_msgs::Header(), ros_img_encoding, cvimg).toImageMsg();
			vpRosImgs.push_back(ros_img);
        }
        
        if (success) {
			for (unsigned int i = 0; i < numCameras; i++)
			{
				 vImgPubs[i].publish(*vpRosImgs[i]);
			}
			ROS_INFO("published.");
		}
		success = true;		
		r.sleep();
    }

    //
    // Stop streaming for each camera
    //
    for (unsigned int i = 0; i < numCameras; i++)
    {
        
		pCameras[i].StopCapture();
        pCameras[i].Disconnect();
    }

    delete[] pCameras;


    return 0;
}
