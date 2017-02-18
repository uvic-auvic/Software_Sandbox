#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <math.h>
#include <turtlesim/Pose.h>

#define banana_cascade "/home/ubuntu/example_ws/src/opencvtest/include/opencvtest/banana_classifier.xml"
#define face_cascade "/home/ubuntu/example_ws/src/opencvtest/include/opencvtest/haarcascade_frontalface_alt.xml"
#define PIXPERMM 84/150 //pixels per mm from P/W
#define FOCAL 84*700/150 //Parameter for specific camera based on calibration. F = PxD/W = PIXPERMM * D
#define WIDTH 150 //Actual width of object mm

using namespace cv;
RNG rng(12345);

class ImageConverter
{
    ros::NodeHandle nh_;
    ros::Publisher pos_pub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    Mat imgFrame_;
    CascadeClassifier banana_classifier;
    CascadeClassifier face_classifier;

public:

    ImageConverter()
        : it_ (nh_)
    {
        image_sub_ = it_.subscribe("/usb_cam/image_raw",1,&ImageConverter::imageCb, this);
        pos_pub_ = nh_.advertise<turtlesim::Pose>("camera1/pose", 1);

        /*if(!banana_classifier.load( banana_cascade))
        {
            std::cout << "Error loading banana cascade." << std::endl;
        }*/
        if(!face_classifier.load( face_cascade))
        {
            std::cout << "Error loading Face cascade." << std::endl;
        }
    }

    ~ImageConverter()
    {
        cv::destroyAllWindows();
    }


    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        this->imageEdgeTracker(cv_ptr);

        int c = cv::waitKey(3);
        if((char)c == 27) ros::shutdown();

    }

    void cascadeTracker(Mat imgThresh)
    {
        //vector<Rect> bananas;
        vector<Rect> faces;
        turtlesim::Pose msg;
        //banana_classifier.detectMultiScale(imgThresh, bananas, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30,30));
        face_classifier.detectMultiScale(imgThresh, faces, 1.1, 5, 0|CV_HAAR_SCALE_IMAGE, Size(30,30));
        /*
        for(int i = 0; i < bananas.size(); i++)
        {
            Point center( (bananas[i].x + bananas[i].width*0.5), (bananas[i].y + bananas[i].height*0.5) );
            ellipse ( imgFrame_, center, Size(bananas[i].height*0.5, bananas[i].width*0.5), 0, 0, 360, Scalar(128,255,255), 4, 8, 0);
        }*/
        for(int i = 0; i < faces.size(); i++)
        {
            Point center( (faces[i].x + faces[i].width*0.5), (faces[i].y + faces[i].height*0.5) );
            ellipse ( imgFrame_, center, Size(faces[i].height*0.5, faces[i].width*0.5), 0, 0, 360, Scalar(255,128,255), 4, 8, 0);
            float distance = ((WIDTH * FOCAL) / (faces[i].width) );
            float xCoord = (center.x * PIXPERMM);
            float yCoord = (center.y * PIXPERMM);
            std::stringstream textss;
            textss << "(" << xCoord << "mm," << yCoord << "mm," << distance <<"mm)";
            putText(imgFrame_, textss.str().c_str(), center, FONT_HERSHEY_PLAIN, 1.5, Scalar(0,0,255));
            msg.x = xCoord;
            msg.y = yCoord;
            msg.theta = distance;
            pos_pub_.publish(msg);
        }
        return;
    }


    void imageEdgeTracker(cv_bridge::CvImagePtr cv_ptr)
    {
        Mat frame(cv_ptr->image);
        Mat imgThresholded;


        imgFrame_ = frame.clone();

        cvtColor(frame, imgThresholded, CV_BGR2GRAY);

        equalizeHist( imgThresholded, imgThresholded);

        cascadeTracker(imgThresholded);

        imshow("imgFrame_", imgFrame_);

        return;
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "imageConverter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
