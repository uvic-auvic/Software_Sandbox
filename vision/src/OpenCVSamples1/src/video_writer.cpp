#include <cv.h>
#include <highgui.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ctime>

#define VIDEO_FILE_LOCATION "/catkin_ws/src/opencvtest/video/"
using namespace cv;

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::CameraSubscriber cam_sub_;
    image_transport::Publisher image_pub_;
    Mat Image_;
    time_t timeNow_, timeLast_;
    int frames;

    VideoWriter outputVideo_;
    bool initFlag_;

    Mat gray_image;

    vector<Point3f> obj;
public:
    ImageConverter()
        : it_ (nh_)
    {
        std::string image_topic = nh_.resolveName("image");
        image_sub_ = it_.subscribe("/crop/image_raw",1,&ImageConverter::imageCb, this);
        //cam_sub_ = it_.subscribeCamera(image_topic, 1, &ImageConverter::camCb, this);
        initFlag_ = true;
        time(&timeLast_);
        frames=0;
    }

    ~ImageConverter()
    {
        cv::destroyAllWindows();
    }

    void camCb(const sensor_msgs::ImageConstPtr& msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
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
        if(initFlag_)
        {
            initFlag_ = false;
            std::stringstream ss;
            ss <<VIDEO_FILE_LOCATION<<"VideoTest.avi";
            outputVideo_.open(ss.str().c_str(),CV_FOURCC('M','J','P','G'), 30, cv_ptr->image.size());
            if(!outputVideo_.isOpened())
            {
                ROS_ERROR("Output Video %s failed to open.", ss.str().c_str());
            }
        }


      this->imageBoardCapture(cv_ptr);
        this->FPSGet();
        cv::waitKey(3);
        return;
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
        if(initFlag_)
        {
            initFlag_ = false;
            std::stringstream ss;
            time_t now = time(0);
            tm *t = localtime(&now);
            ss <<getenv("HOME")<<VIDEO_FILE_LOCATION<<"AUVic-"<<1900+t->tm_year<<","<<1+t->tm_mon<<","<<t->tm_mday<<"-"<<t->tm_hour<<":"<<t->tm_min<<":"<<t->tm_sec<<".avi";
            outputVideo_.open(ss.str().c_str(),CV_FOURCC('M','J','P','G'), 16.5, cv_ptr->image.size());
            if(!outputVideo_.isOpened())
            {
                ROS_ERROR("Output Video %s failed to open.", ss.str().c_str());
            }
        }


      this->imageBoardCapture(cv_ptr);
        this->FPSGet();
        cv::waitKey(3);
    }

    void imageBoardCapture(cv_bridge::CvImagePtr cv_ptr)
    {
        outputVideo_ << Mat(cv_ptr->image);
        return;
    }

    void FPSGet()
    {
        time(&timeNow_);
        frames++;
        double seconds = difftime(timeNow_, timeLast_);

        if(seconds >= 1.0)
        {
            double fps;
            time(&timeLast_);
            fps = frames/seconds;
            frames = 0;
            std::cout<<"FPS:"<<fps<<std::endl;
        }

        return;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imgRedTracker");
    ImageConverter ic;
    ros::spin();
return 0;
}
