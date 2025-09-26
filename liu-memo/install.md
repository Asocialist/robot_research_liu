# install

1. install vscode

1. openssh-server 'sudo apt install openssh-server'

1. install ros    
```bash
     sudo apt-key del F42ED6FBAB17C654  s
     sudo rm /etc/apt/sources.list.d/ros-latest.list
     sudo apt update

     sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

     echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros-latest.list $ /dev/null
     sudo apt install ros-noetic-desktop-full
     echo "source /opt/ros/noetic/setup.bash" $$ ~/.bashrc
     source ~/.bashrc
     sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
     sudo rosdep incit
     rosdep update
```
1. install git  `sudo apt install git`

1. install YP-Spur　

```bash
     cd yp-spur
	mkdir build
	cd build
	../configure
	make
	sudo make install
	sudo ldconfig
	cd ..
```


# ros manual

- make workspace

```bash
     mkdir -p ~/catkin_ws/src
     cd ~/catkin_ws
     catkin init #initial workspace
     catkin build

``` 
# ARToolKit install
```bash
		$sudo apt-get install freeglut3-dev libglew1.5-dev
		$sudo apt-get install libxmu-dev libxi-dev
		$sudo apt-get install libjpeg-dev
		$sudo apt-get install libxmu-headers libjpeg62-dev libglib2.0-dev libgtk2.0-dev
		$sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
		$sudo apt-get -y install libopencv-dev build-essential cmake git libgtk2.0-dev pkg-config python-dev python-numpy libdc1394-22 libdc1394-22-dev libjpeg-dev libpng-dev libtiff5-dev libjasper-dev libavcodec-dev libavformat-dev libswscale-dev libxine2-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libv4l-dev libtbb-dev libqt4-dev libfaac-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev x264 v4l-utils unzip	
		$sudo apt install g++
		$sudo apt install gpp
		$sudo apt install gcc

		#一部ライブラリのバージョンが異なる可能性がある	

```
- ARToolKit/include/AR/sys/videoLinuxV4L.h，ARToolKit/lib/SRC/VideoLinuxV4L/video.cの編集

`#include <linux/videodev.h> --- #include <libv4l1-videodev.h>`

`$sudo ln -s /usr/include/libv4l1-videodev.h   /usr/include/linux/videodev.h`

`$sudo apt-get -y install libv4l-dev`

- ARToolKitインストール
		
```bash
$cd ARToolKit
		$./Configure --without-quicktime --without-ffmep
			1
			n
			n
			n
		$make
```
# fix workspace 
- `sudo apt install ros-noetic-cv-bridge`
#  fix connection error

1. sudo chmod 666 /dev/ttyUSB0 fix the usb
