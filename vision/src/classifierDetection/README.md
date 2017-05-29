# Classifier Detection with OpenCV

The purpose of this README.md and associacted classifierDetectionSample.cpp and classifier files(.xml) provided is to show how to write a 
simple piece of code using OpenCV to provide classifier detection using lbp or haar classifiers.

## How to get started

*Important: This tutorial is done without the use of ROS for simplicity and to make learning OpenCV as simple as possible.*

1. Before starting this tutorial make sure that you have OpenCV installed on your computer. 
2. Please take a look at the comments in the code before advancing.
3. To test the file run the following commands:
  - $ cmake .
  - $ make
  - $ ./classifierDetectionSample
  
## Code explained 

The video feed from your webcam is passed into a frame object which can then be used by functions in the OpenCV library to perform
classifier detection. Each frame is then converted into gray scale before classifier detection can be performed. On the detection frame,
lines are draw around the object and displayed in video feed on screen for user. Accuracy with the banana classifier is not great
but using the face and eye classifier provided by OpenCV in their examples one can see an example of accurate detection. 

*Important: Make sure to comment out banana and face detection sections respectively in classifierDetectionSample.cpp to perform detection for 
desired classifier object.*
