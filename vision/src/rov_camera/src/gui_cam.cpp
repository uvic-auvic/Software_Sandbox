#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <ctime>

#define VIDEO_FILE_LOCATION "/ROV02/video/"
using namespace cv;
RNG rng(12345);

class ImageConverter
{
    ros::NodeHandle nh_;
    ros::Subscriber gui_sub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    bool recordingMode;

    VideoWriter outputVideo_;

    cv::Size imageSize;

    bool initFlag;

public:

    ImageConverter()
        : it_ (nh_)
    {
        image_sub_ = it_.subscribe("/usb_cam/image_raw",1,&ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/GUI/camera", 1);
        gui_sub_ = nh_.subscribe("/GUI/recordingMode", 1, &ImageConverter::controlCb, this);

        recordingMode = false;

        initFlag = true;
    }

    ~ImageConverter()
    {
        cv::destroyAllWindows();
    }

    void controlCb(const std_msgs::Int16 msg)
    {
        if(msg.data == 0)
        {
            time_t now = time(0);
            std::stringstream ss;
            tm *ltm = localtime(&now);
            ss <<(std::string(getenv("HOME"))+VIDEO_FILE_LOCATION)<<"AUVic-y"<<1900+ltm->tm_year<<"-m"<<1+ltm->tm_mon<<"-d"<<ltm->tm_mday<<"-h"<<ltm->tm_hour<<"-m"<<ltm->tm_min<<"-s"<<ltm->tm_sec<<".avi";
            outputVideo_.open(ss.str().c_str(), CV_FOURCC('M','J','P','G'), 30, imageSize);
            recordingMode = true;
            if(!outputVideo_.isOpened())
            {
                ROS_ERROR("Output video %s failed to open", ss.str().c_str());
                recordingMode = false;
            }
        }
        else
        {
            recordingMode = false;
            outputVideo_.release();
        }
        return;
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            if(initFlag == true){
                imageSize = cv_ptr->image.size();
                initFlag = false;

            }

        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        if(recordingMode)
        {
            outputVideo_ << cv_ptr->image;
        }

        image_pub_.publish(cv_ptr->toImageMsg());
    }


};


int main(int argc, char** argv) {
    ros::init(argc, argv, "imageConverter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
