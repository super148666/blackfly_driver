#include "FlyCapture2.h"
#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

using namespace FlyCapture2;
using namespace std;

int numCameras;
vector<string> topicNames;
vector<int> vSerials;
PixelFormat imgFormat;
int spinRate;
vector<ros::Publisher> vImgPubs;
vector<sensor_msgs::ImagePtr> vpRosImgs;
string ros_img_encoding;

void PrintError(Error error) { error.PrintErrorTrace(); }

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
    Error error = cam.ReadGVCPRegister(k_GVCPCapabilityAddr, &regVal);
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
    // std::string pkg_path = ros::package::getPath("blackfly_driver")+"/";
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
    string strFormat = "";
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
    if(!nh.getParam(paramName, spinRate)) {
        ROS_ERROR_STREAM("Please assign " << paramName << ".\n" <<  
                         "Aborting.");
        exit(-1);
    }
    ROS_INFO_STREAM(paramName << ": " << spinRate << ".");

    ROS_INFO("Done read parameters.");
}


int main(int argc, char ** argv)
{
	ros::init(argc, argv, "flycapture_driver");
	ros::NodeHandle nh;
	ReadParams(nh);
	
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
	
	// init publishers
    for (int i = 0; i < numCameras; i++) {
        ros::Publisher pub = nh.advertise<sensor_msgs::Image>(topicNames[i], 1);
        vImgPubs.push_back(pub);
    }
    ROS_INFO_STREAM("Done init publisher. size: " << vImgPubs.size());
	
    PrintBuildInfo();

    Error error;

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

    cout << "Number of cameras detected: " << numAllCameras << endl;

    //
    // Check to make sure at least two cameras are connected before
    // running example
    //
    if (numAllCameras < numCameras)
    {
        cout << "Insufficient number of cameras." << endl;
        cout << "Make sure at least " << numCameras << " cameras are connected for example to "
                "run."
             << endl;
        cout << "Press Enter to exit." << endl;
        cin.ignore();
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
            cout << "Press Enter to exit." << endl;
            delete[] pCameras;
            cin.ignore();
            return -1;
        }

        // Connect to a camera
        error = pCameras[i].Connect(&guid);
        if (error != PGRERROR_OK)
        {
            PrintError(error);
            delete[] pCameras;
            cout << "Press Enter to exit." << endl;
            cin.ignore();
            return -1;
        }

        // Get the camera information
        CameraInfo camInfo;
        error = pCameras[i].GetCameraInfo(&camInfo);
        if (error != PGRERROR_OK)
        {
            PrintError(error);
            delete[] pCameras;
            cout << "Press Enter to exit." << endl;
            cin.ignore();
            return -1;
        }

        PrintCameraInfo(&camInfo);

        // Turn trigger mode off
        TriggerMode trigMode;
        trigMode.onOff = false;
        error = pCameras[i].SetTriggerMode(&trigMode);
        if (error != PGRERROR_OK)
        {
            PrintError(error);
            delete[] pCameras;
            cout << "Press Enter to exit." << endl;
            cin.ignore();
            return -1;
        }

        // Start streaming on camera
        error = pCameras[i].StartCapture();
        if (error != PGRERROR_OK)
        {
            PrintError(error);
            delete[] pCameras;
            cout << "Press Enter to exit." << endl;
            cin.ignore();
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

            cout << "get image from camera " << i << "." << endl;
			error = image.Convert(imgFormat, &convertedImage);
			if (error != PGRERROR_OK)
			{
				PrintError(error);
				return -1;
			}
			unsigned int rowBytes = (double)convertedImage.GetReceivedDataSize()/(double)convertedImage.GetRows();
			cv::Mat cvimg = cv::Mat(convertedImage.GetRows(), convertedImage.GetCols(), CV_8UC3, convertedImage.GetData(),rowBytes);
			
			sensor_msgs::ImagePtr ros_img = cv_bridge::CvImage(std_msgs::Header(), ros_img_encoding, cvimg).toImageMsg();
			vpRosImgs.push_back(ros_img);
        }
        if (success) {
			for (unsigned int i = 0; i < numCameras; i++)
			{
				 vImgPubs[i].publish(*vpRosImgs[i]);
			}
			cout << "published." << endl;
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

    cout << "Press Enter to exit..." << endl;
    cin.ignore();

    return 0;
}
