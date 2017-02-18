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

#define lbpface_cascade "/home/googly/catkin_ws/src/opencvtest/include/opencvtest/lbpcascade_frontalface.xml"
#define buoy_cascade "/home/googly/catkin_ws/src/opencvtest/include/opencvtest/buoy_cascade.xml"
#define FOCAL 106*508/190 // FOCAL Length for 1900x1200 Aspect Ratio 19:12 F = PxD/W
#define WIDTH 190 //Actual width of object mm

using namespace cv;
RNG rng(12345);

class ImageConverter
{
    ros::NodeHandle nh_;
    ros::Publisher pos_pub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    Mat imgFrame_;
    CascadeClassifier face_classifier;


    int frames;
    int height_, ScaleFactor_, minNeighbours_;
    double fps;
    time_t timeLast_, timeNow_;

public:

    ImageConverter()
        : it_ (nh_)
    {
        image_sub_ = it_.subscribe("/usb_cam/image_raw",1,&ImageConverter::imageCb, this);
        pos_pub_ = nh_.advertise<turtlesim::Pose>("camera1/pose", 1);
        height_ = 480;
        ScaleFactor_ = 40;
        minNeighbours_ = 5;
        if(!face_classifier.load( buoy_cascade))
        {
            std::cout << "Error loading Face cascade." << std::endl;
        }

        namedWindow("Control");
        createTrackbar("Scale factor", "Control", &ScaleFactor_, 100);
        createTrackbar("minNeighbours","Control", &minNeighbours_, 10);

        time(&timeLast_);
        frames = 0;
        fps = 0;
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
            time(&timeNow_);
            frames++;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        this->imageEdgeTracker(cv_ptr);
        this->FPSGet();
        int c = cv::waitKey(3);
        if((char)c == 27) ros::shutdown();
    }

    void FPSGet()
    {
        double seconds = difftime(timeNow_, timeLast_);
        if(seconds >= 2.0)
        {
            time(&timeLast_);
            fps = frames/seconds;
            frames = 0;
            std::cout<<"FPS:"<<fps<<" D:"<< height_<<"x"<<height_<<std::endl;
        }
        return;
    }

    void cascadeTracker(Mat imgThresh)
    {
        vector<Rect> faces;
        turtlesim::Pose msg;
        double scaleFactor = 1+ScaleFactor_/100.0;
        if(scaleFactor <= 1.05)
            scaleFactor = 1.05;
        face_classifier.detectMultiScale(imgThresh, faces, scaleFactor, minNeighbours_, 0|CV_HAAR_SCALE_IMAGE, Size(5,5), Size(imgThresh.cols/2, imgThresh.rows/2));

        for(int i = 0; i < faces.size(); i++)
        {
            Point center( (faces[i].x + faces[i].width*0.5), (faces[i].y + faces[i].height*0.5) );
            ellipse ( imgFrame_, center, Size(faces[i].height*0.5, faces[i].width*0.5), 0, 0, 360, Scalar(255,128,255), 4, 8, 0);
            float focal = FOCAL; // *(height_)/(1200);
            float distance = ((WIDTH * focal) / (faces[i].width) ) / 10.0;
            float xCoord = (((center.x - imgFrame_.size().width/2) * distance) / focal );
            float yCoord = (((imgFrame_.size().height/2 - center.y) * distance) / focal );

            std::stringstream textss;
            textss <<"("<<" "<< round(xCoord) << "cm," << round(yCoord) << "cm," << round(distance) <<"cm)";
            putText(imgFrame_, textss.str().c_str(), center, FONT_HERSHEY_PLAIN, 1.2, Scalar(0,0,255), 2);
            msg.x = xCoord / 100.0;
            msg.y = yCoord / 100.0;
            msg.theta = distance / 100.0;
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
        putText(imgFrame_, "(0,0)", Point(imgFrame_.size().width/2, imgFrame_.size().height/2), 1, FONT_HERSHEY_PLAIN,Scalar(0,0,255));
        line(imgFrame_,Point(imgFrame_.size().width/2, 0), Point(imgFrame_.size().width/2, imgFrame_.size().height), Scalar(0,0,255));
        line(imgFrame_,Point(0, imgFrame_.size().height/2), Point(imgFrame_.size().width, imgFrame_.size().height/2), Scalar(0,0,255));

        imshow("imgFrame_", imgFrame_);

        return;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "banana_test");
    ImageConverter ic;
    ros::spin();
    return 0;
}
 
