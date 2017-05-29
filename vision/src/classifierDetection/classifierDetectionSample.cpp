//Author: Keifer Edelmayer
/*
Program Description: With use of webcam, program is able to take classifier provided and perform detection on selected classifier. Code is currently set up to use the banana classifier found in the auvic repo. Certain sections of code marked below can be commented/uncommeted to also perform facial detection using the classifier provided by openCV which can be found in our repo or in OpenCV's samples.
*/

#include <iostream>
#include <stdio.h>
#include "opencv2/opencv.hpp"

using namespace cv; using namespace std;

void detection( Mat frame );

/*****************************************

------------Face Detection Section--------

******************************************/

/*
String face_cascade_name = "haarcascade_frontalface_alt.xml";
String eyes_cascade_name = "haarcascade_eye_tree_eyeglasses.xml";
CascadeClassifier face_cascade;
CascadeClassifier eyes_cascade;
String window_name = "Capture - Face detection";
*/

/******************************************

-------------------------------------------

******************************************/


/******************************************

-----------Banana Detection Section--------

*******************************************/

String banana_cascade_name = "banana_classifier.xml";
CascadeClassifier banana_cascade;
String window_name = "Capture - Banana detection";

/******************************************

-------------------------------------------

******************************************/

int main(int argc, char** argv){
    VideoCapture cap;

/******************************************

-----------Banana Detection Section--------

*******************************************/

    if( !banana_cascade.load( banana_cascade_name ) ){
        printf("--(!)Error loading you shit\n");
        return -1;
    }

/******************************************

-------------------------------------------

******************************************/
 
/*****************************************

------------Face Detection Section--------

******************************************/

/* 
    if( !face_cascade.load( face_cascade_name ) ){
        printf("error1");
        return -1;
    }

    if( !eyes_cascade.load( eyes_cascade_name ) ){
        printf("error2");
        return -1;
    } 
*/

/******************************************

-------------------------------------------

******************************************/

    if(!cap.open(0)) //checks to see if there is video feed, closes if none
        return 0;

    for(;;){ //runs forever taking stream into frame format
        Mat frame;
        cap >> frame;
        if( frame.empty() )
            break;
        
        detection( frame ); //function for detection using classifier
      

        if( waitKey(10) == 27 ) //if esc key is pressed program will close
            break;
    }
    return 0;
}

void detection( Mat frame ){

/******************************************

-----------Banana Detection Section--------

*******************************************/

    std::vector<Rect> bananas;
    Mat frame_gray;
    cvtColor( frame, frame_gray, COLOR_BGR2GRAY ); //converts frame to grayscale before detection can occur
    equalizeHist( frame_gray, frame_gray ); //smooths frame for img processing
    banana_cascade.detectMultiScale( frame_gray, bananas, 1.1 , 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) ); //detection of the classifier object

    for( size_t i = 0; i < bananas.size(); i++ ){ //loop draws elipse around detected objects to display on screen
        Point center( bananas[i].x + bananas[i].width/2, bananas[i].y + bananas[i].height/2 );
        ellipse( frame, center, Size( bananas[i].width/2, bananas[i].height/2 ), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 ); 
    }
    imshow( window_name, frame );  

/******************************************

-------------------------------------------

******************************************/
 

/*****************************************

------------Face Detection Section--------

******************************************/


/*
//example code for facedetection from the OpenCV examples
   
    std::vector<Rect> faces; Mat frame_gray;
    cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );

//-- Detect faces

    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );
    for ( size_t i = 0; i < faces.size(); i++ )
    {
        Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
        ellipse( frame, center, Size( faces[i].width/2, faces[i].height/2 ), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
        Mat faceROI = frame_gray( faces[i] );
        std::vector<Rect> eyes;
//-- In each face, detect eyes

        eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CASCADE_SCALE_IMAGE, Size(30, 30) );
        for ( size_t j = 0; j < eyes.size(); j++ )
        {
            Point eye_center( faces[i].x + eyes[j].x + eyes[j].width/2, faces[i].y + eyes[j].y + eyes[j].height/2 );
            int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
            circle( frame, eye_center, radius, Scalar( 255, 0, 0 ), 4, 8, 0 );
        }
    }

//-- Show what you got

    imshow( window_name, frame );
*/

/******************************************

-------------------------------------------

******************************************/

}
