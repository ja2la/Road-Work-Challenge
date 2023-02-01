## Table of contents
* [General info](#general-info)
* [Required Hardware](#required-hardware)
* [Requirements](#requirements)
* [How to launch the project](#how-to-launch-the-project)

## General info
This project tries to meet the requirements of the challenge 'Road Work' at the Cybathlon competion (2024).
	
## Required Hardware
* RGB-D camera (in this project an Asus Xtion Pro was used)
* A computer with ROS, Python and a speaker

## Requirements
# Python :
* OpenCV
* Numpy
* Playsound
* Time

# ROS :
* Openni2

## How to launch the project
1. Open a terminal
```
$ roscore
```

2. Open another terminal
Launch the camera
```
$ roslaunch openni_launch openni.launch
```

3. Open another terminal
Run the Image Processing Node

First set the red color mask :
```
$ cd catkin_make/src/Image_Processing_Algorithm/scripts
$ chmod +x SetHSV.py
$ rosrun Image_Processing_Algorithm SetHSV.py
```
Keep the values of trackbars and set them in the Algorithm.py file

Then you can run the complete algorithm
```
$ cd catkin_make/src/Image_Processing_Algorithm/scripts
$ chmod +x Algorithm.py
$ rosrun Image_Processing_Algorithm Algorithm.py
```

5. Open another terminal
Run the Vocal Feedback Node

```
$ cd catkin_make/src/Vocal_Feedback/scripts
$ chmod +x Vocal_Instructions.py
$ rosrun Vocal_Feedback Vocal_Instructions.py
```

## Group Members
* Juliette Faure
* Esra Gudum
* Jade La
* Lauraine Tiogang Nanko
