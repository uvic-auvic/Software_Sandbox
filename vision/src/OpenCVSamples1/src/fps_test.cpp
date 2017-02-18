#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <time.h>

using namespace cv;

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    time_t timeLast_, timeNow_;
    int frames;
    double fps;

public:
    ImageConverter()
        : it_ (nh_)
    {
        image_sub_ = it_.subscribe("/camera/image_raw",1,&ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/crop/image_raw",1);
        frames = 0;
        fps = 0;
        time(&timeLast_);

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
        this->FPSGet();
        this->imageCrop(cv_ptr);
    }

    void imageCrop(cv_bridge::CvImagePtr cv_ptr)
    {
        Mat frame(cv_ptr->image);

        Rect roi(Point(350,0),Size(1200,1200));
        cv_ptr->image = frame(roi);

        image_pub_.publish(cv_ptr->toImageMsg());
        return;
    }

    void FPSGet()
    {
        time(&timeNow_);
        frames++;
        double seconds = difftime(timeNow_, timeLast_);

        if(seconds >= 1.0)
        {
            time(&timeLast_);
            fps = frames/seconds;
            frames = 0;
            std::cout<<"FPS:"<<fps<<std::endl;
        }

        return;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fisheye_node");
    ImageConverter ic;
    ros::spin();
    return 0;
}
