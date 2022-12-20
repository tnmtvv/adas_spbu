# Description
This branch contains a human pose estimation false positive filter based on OpenPose. Also, the scripts for getting the models were taken from [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose).

## Requirements:
You should first install Cuda version 11 or higher, CuDNN version 8 or higher, gcc version 10 or higher, and cmake version 3 or higher.

## Usage guide
To demonstrate the work of this project, you need to 
##### Step №1: Install OpenCV latest version
You need to install opencv using this [guide](https://docs.opencv.org/4.x/d0/d3d/tutorial_general_install.html).
Also if you do it on linux, you should compile OpenCV with a GUI backend and video reader such as GTK and FFmpeg.
##### Step №2: Install OpenPose
You need to install opencv using this [guide](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation/0_index.md), then go to the build directory of OpenPose and run this command if you are on Linux/Mac OS:
```shell
sudo make install
```
If you are on windows then set the environment variables path to build OpenPose.
##### Step №3: Clone the Project
```shell
git clone https://github.com/osechkina-masha/adas_spbu.git
cd adas_spbu
git checkout pedestrian_detection
```
##### Step №4: Getting models
If you are on Windows run the models/getModels.bat file, if you are on Linux/Mac OS run the models/getModels.sh file.
After that, you must also replace the model in the folder model/body_25 with the model from this [link](https://www.dropbox.com/s/03r8pa8sikrqv62/pose_iter_584000.caffemodel?dl=0), otherwise there will be an error to parse NetParameter file.
##### Step №5: Project Launch
If you are on Windows using cmake-gui, configure the project for Visual Studio, build and run the *.sln file with Visual Studio.
On Linix/Mac OS, build and run the project with these commands:
```shell
cmake .
make
./adas_spbu your_video_file.mp4
```

## Authors
[@Grigory-Aseev](https://github.com/Grigory-Aseev)