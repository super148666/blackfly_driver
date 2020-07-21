# blackfly_driver
blackfly ros driver based on spinnaker and flycapture 2 sdk. Developed for UWA REV SAE and nUWAy autonomous driving project

# Dependencies

1. Please install flycapture 2 SDK from https://www.flir.com.au/products/flycapture-sdk/
2. install following dependencies in terminal
```
sudo apt-get update
sudo apt-get install -y libavcodec-ffmpeg56 libavformat-ffmpeg56 libswscale-ffmpeg3 libswresample-ffmpeg1 libavutil-ffmpeg54 libusb-1.0-0 build-essential git
```

# Install
clone package into `src` under a ROS workspace
```
git clone https://github.com/super148666/blackfly_driver.git
```

# Compile
Under root of ROS workspace
```
catkin_make
source ./devel/setup.bash
```

# Config
edit `nuway.yaml` under folder `params`
```yaml
/num_of_cameras: 2
/topic1: '/right_image'
/serial1: '15383846'
/serial2: '15420581'
/topic2: '/left_image'
#Pixel Format: mono/rgb/bgr/rgba/bgra + 8/16
/format: 'rgb8'
#Color Processing Algorithm: DEFAULT, NO_COLOR_PROCESSING, NEAREST_NEIGHBOR, EDGE_SENSING, HQ_LINEAR, RIGOROUS, IPP, DIRECTIONAL_FILTER
/algorithm: 'NEAREST_NEIGHBOR'
/spin_rate: 15
/packet_size: 8100
/packet_delay: 6250
#offset: this is to set the start pixel of ROI
/offsetX: 0
/offsetY: 0
#Image Size: max (1920, 1200), 2x2 binning(860, 600)
/width: 860
/height: 600

/image_mode: 1
#Calibration - set to false until the parameters in calib.yml are updated properly
/calibrate: false
/absolute_path: false
/file_path: 'params/calib.yml'
```
Note: to find the best value for above configuration, please use the `FlyCap2` from `flycapture 2 SDK`

# Launch
after compiling the package
```
roslaunch blackfly_driver image_publisher_nuway.launch
```