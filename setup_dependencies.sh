#!/bin/bash
# /*  
#     This code was developed by Chao Zhang for using with FLIR blackfly camera
# */

# /*
#     This file is part of blackfly_ros_driver

#     blackfly_ros_driver is free software: you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by
#     the Free Software Foundation, either version 3 of the License, or
#     (at your option) any later version.

#     blackfly_ros_driver is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.

#     You should have received a copy of the GNU General Public License
#     along with blackfly_ros_driver.  If not, see <https://www.gnu.org/licenses/>.
# */

# /*-*-C++-*-*/
# /**
#  * @file        setup_dependencies.sh
#  * @author      Chao Zhang
#  * @created     June 21, 2018
#  * @last edit   June 21, 2018
#  * @breif       Setup the environment for using blackfly_ros_driver
#  * 
#  * @attention   Copyright (C) 2018
# */

sudo apt-get update
sudo apt-get install -y libavcodec-ffmpeg56 libavformat-ffmpeg56 \
                        libswscale-ffmpeg3 libswresample-ffmpeg1 \
                        libavutil-ffmpeg54 libusb-1.0-0
