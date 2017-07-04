# Edge Detection with OpenCV

The purpose of this README.md associated edgeDetectionSample.cpp code and CMakeLists.txt file provided is to show the user how to write a 
simple piece of code using Opencv to perform edge Detection.

## How to get started

*Important: This tutorial is done without the use of ROS for simplicity and to make learning OpenCV as simple as possible.*

1. Before starting this tutorial make sure that you have OpenCV installed on your computer.
2. Please take a look at the comments in the code before advancing.
3. To test the file run the following commands:
    - $ cmake .
    - $ make
    - $ ./edgeDetectionSample
    
## Code explained 

Edge dection is performed using OpenCV with the canny edge detector developed by John F. Canny. The basics steps of the algorithm can be 
broken down into the five simplified steps below:

1. Filter noise from the image
2. Find the intesity of the Gradient
3. Non-maximum supression ( removal of pixels not in edge )
4. Threshhold filtering:

        -Canny uses two thresholds to filter

        -Upper threshold: All pixel gradients above accepted
        
        -Lower threshold: All pixel gradients below rejected
         
        -Inbetween thresholds: Only accepted if connected to pixel above upper threshold
        
5. Accepted pixels printed back onto black image producing an edge only image

The program file opens video feed from your webcam, which is then passed into a frame to be converted to grayscale. The grayscale image
is then filtered for noise and then passed to the OpenCV Canny function for edge detection. The results from the canny function are then 
printed on top of a black frame and returned for viewing to the user. The accepted threshold limit can be controlled using a created 
sliderbar for threshold control.






