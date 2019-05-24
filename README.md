# IMAGE_PROCESSING

This package contains multiple python scripts to process images and calibrate a camera.The processed image returns the x, y and theta of the center point of an object. This centerpoint is measured from the centerpoint of the camera. The x, y and theta will be used to create a TF, so a robotic arm can pick and place the object.

If you want to learn more of how the vision systems work, please check out our **wiki**.

## Getting Started

This package only contains python2 scripts and make use of OpenCV-3.3.1-dev and numpy 1.11.0.

### Prerequisites

Assuming python2 and numpy are already installed, you only need to install OpenCV. You can install OpenCV by running the following lines:

```
$ sudo apt-get install python-opencv
$ python2
import cv2 as cv
print(cv.__version__)
```
The last line will result in printing your installed OpenCV version. For more information about installin OpenCV, check the [OpenCV tutorial](https://docs.opencv.org/3.4/d2/de6/tutorial_py_setup_in_ubuntu.html)

### Installing

To install the image_processing package in your catkin workspace, you will need to run the following lines:
```
cd catkin_ws/src
git clone **add link**
catkin_make
```

## Running the tests

To test the code you need to run the following lines:
'''
cd /home/user_name/catkin_ws/src/image_processing/src
python2 post_processing_test.py 
'''
You should recieve some output like:
```
area 6 blur 126 lower 75 upper 251
pix length: 77.9999847412, pix width: 56.9999923706
Object lenght: 48.7499904633 mm, Object width: 35.6249952316 mm
True
[['Bolt', -175.62500476837158, 108.43750476837158, 0.6531277687845073]]
True
```

**note**: You might need to change the cv2.VideoCapture(0) to your desired camera input. You can check your input as followed:
```
ls /dev/video*
```
**note**: If you want to show the processed image, edit the post_processing_test.py file. 
change *build_center = self.test.build_center("Bolt",(0,0,640,480),frame,False)* **to** *build_center = self.test.build_center("Bolt",(0,0,640,480),frame,True)*
 

## Authors

* **Jeroen Bongers** - *in name of RoboHub Eindhoven* - [RoboHub Eindhoven website](https://robohub-eindhoven.nl/)



