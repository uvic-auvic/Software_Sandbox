//Author: Keifer Edelmayer
/*
Program Description: With use of webcam, program is able to perform color detection on objects placed before the webcam. An additional control window with sliders
for the RGB values has been created to allow for color detection of desired color.
*/
#include <iostream>
#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    VideoCapture cap;
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if(!cap.open(0))
        return 0;

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called 'Control"

    int iLowH = 0;
    int iHighH = 179;

    int iLowS = 0; 
    int iHighS = 255;

    int iLowV = 0;
    int iHighV = 255;

    //Create trackbars in "Control" window
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);
   
    for(;;) //infitie loop, runs till break
    {
        Mat frame; 
        cap >> frame; //camera footage being feed into frame
        if( frame.empty() ) break; // end of video stream
    
        Mat imgHSV;
        cvtColor(frame, imgHSV, COLOR_BGR2HSV); //take the img and convert to HSV to allow for color detection to be performed
        Mat imgThresholded;
        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //takes values from the sliders and filters colors currently not in range

        //noise math for the imgage to be improve quality below
    
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

        //morphological closing (fill small holes in the foreground)
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    
        imshow("original img", frame); //function to show webcam feed
        imshow("thresholded img", imgThresholded);

        if( waitKey(10) == 27 ) break; // stop capturing by pressing ESC 
    }
    // the camera will be closed automatically upon exit
    // cap.close();
    return 0;
}
