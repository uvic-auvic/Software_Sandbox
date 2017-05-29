# Color Detection with OpenCV 

The purpose of this README.md and associated colorDetectionSample.cpp code provided is to show the user how to write a simple piece of code using OpenCV to perform color detection.

## How to get started

*Important: This tutorial is done without the use of ROS for simplicity and to make learning OpenCV as simple as possible.*

1. Before starting this tutorial make sure that you have OpenCV installed on your computer.
2. Please take a look at the comments in the code before advancing.
3. To test the file run the following commands:
    - $ cmake .
    - $ make
    - $ ./colorDetectionSample

## Code explained

The video feed from your webcam is passed into a frame object which can then be used by functions in the OpenCV libary to perform color detection. Each frame is then converted into HSV before color detection can be performed. The threshold limits are controled by the sliders, which control which RGB values that appear within the accepted range. With "white" showing values within the range and color values outside the range appearing "black" creating a inverted silhouette image of the color or range we are detecting. 
