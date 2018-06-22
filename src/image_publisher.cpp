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
 * @file        image_publisher.cpp
 * @author      Chao Zhang
 * @created     June 21, 2018
 * @last edit   June 21, 2018
 * @brief       implements
 * 
 * @attention   Copyright (C) 2018
*/

#include <ros/ros.h>
#include "blackfly_driver.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "blackfly_driver");
    ros::NodeHandle nh;

    blackfly_driver bd(nh);

    bd.configure();
    //bd.printCameraInfo();
    bd.spin();

    return 0;
}
