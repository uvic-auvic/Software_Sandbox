#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <math.h>
#include "rov_camera/CameraControl.h"

#define face_cascade "/home/ubuntu/example_ws/src/opencvtest/include/opencvtest/haarcascade_frontalface_alt.xml"

using namespace cv;

class ImageConverter
{
    Mat imgFrame_;
    CascadeClassifier face_classifier;
public:

    ImageConverter()
    {
        if(!face_classifier.load( face_cascade))
        {
            std::cout << "Error loading Face cascade." << std::endl;
        }
    }

    ~ImageConverter()
    {
        cv::destroyAllWindows();
    }

    void cascadeTracker(Mat imgThresh)
    {
        vector<Rect> faces;
        face_classifier.detectMultiScale(imgThresh, faces, 1.1, 7);
        for(int i = 0; i < faces.size(); i++)
        {
            Point center( (faces[i].x + faces[i].width*0.5), (faces[i].y + faces[i].height*0.5) );
            ellipse ( imgFrame_, center, Size(faces[i].height*0.5, faces[i].width*0.5), 0, 0, 360, Scalar(255,128,255), 4, 8, 0);
        }
        return;
    }


    void imageEdgeTracker(std::string file_name)
    {
        Mat frame;
        Mat imgThresholded;

        frame = imread(file_name, CV_LOAD_IMAGE_COLOR);
        if(frame.data)
        {
            imshow("Frame", frame);
            imgFrame_ = frame.clone();
            cvtColor(frame, imgThresholded, CV_BGR2GRAY);
            equalizeHist( imgThresholded, imgThresholded);
            cascadeTracker(imgThresholded);

            imshow("imgFrame_", imgFrame_);
            while(1);
        }
        else
            std::cout << "Could not load or find image file.";
        return;
    }
};


int main(int argc, char** argv) {

    ImageConverter ic;

    ic.imageEdgeTracker("/home/ubuntu/example_ws/src/opencvtest/include/opencvtest/obama.jpeg");

    return 0;
}
