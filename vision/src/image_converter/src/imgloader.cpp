#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

int main(int argc, const char** argv) {
    Mat img = imread("/home/kyle/catkin/src/opencvtest/src/img.jpeg", CV_LOAD_IMAGE_UNCHANGED);

    if(img.empty()){
        //img is not found
        cout << "Image cannot be found or opened..." << endl;
        return -1;
    }

    namedWindow("MyWindow",CV_WINDOW_AUTOSIZE);
    imshow("MyWindow", img);

    waitKey(0);
    destroyWindow("MyWindow");

    return 0;
}
