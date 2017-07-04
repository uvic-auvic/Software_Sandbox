//Author: Keifer Edelmayer
/*
Program Description: With use of webcam, program is able to perform edge detection on 
objects within frame of webcam. An additional control window with slider control for 
threshold levels accepted has been created to allow for filtering of desired level.
*/
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "opencv2/opencv.hpp"

using namespace cv; using namespace std;

void edgeDetection( Mat frame );

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
//char* window_name = "Edge Map";


int main(int argc, char** argv){
    VideoCapture cap;
    // open the default camera, use something diffrent from 0 otherwise;
    // Check videocapture documentation
    if(!cap.open(0))
        return 0;
    
    namedWindow("Control", CV_WINDOW_AUTOSIZE);    
    
    //create a trackbar for threshold control for edge detection
    cvCreateTrackbar( "Min Threshold:", "Control", &lowThreshold, max_lowThreshold );

    for(;;){  //infinite loop, runs till no more frames or key break
        Mat frame;
        cap >> frame;
        if( frame.empty() )
            break;

       edgeDetection( frame );

        if( waitKey(10) == 27 )
            break; 
    }
    return 0;
}

void edgeDetection( Mat frame ){
    Mat frame_gray, dst, detected_edges; //create another frame for grayscale conversion
   
    //creation of a matrix of same size and type
    dst.create( frame.size(), frame.type() ); 
    cvtColor( frame, frame_gray, COLOR_BGR2GRAY ); 

    //blur of grayscale img  to filter noise
    blur( frame_gray, detected_edges, Size(3,3) ); 

    //OpenCV edge detection alg function
    Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size);

    //fill the image with a black backround 
    dst = Scalar::all(0);

    //fill in all accepted edge pixels on top of black img
    frame.copyTo( dst, detected_edges);

    //display edge detection video feed back to user
    //imshow( window_name, dst );
    imshow( "threshold img", dst);
}
