#include <cv.h>
#include <highgui.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace cv;

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    Mat Image_;

    int numCornersHor, numBoards, numCornersVer, numSquares;
    int successes;
    Size board_sz;

    vector<vector<Point3f> > object_points;
    vector<vector<Point2f> > image_points;
    vector<Point2f> corners;
    Mat gray_image;
    //capture >> image;
    vector<Point3f> obj;
public:
    ImageConverter()
        : it_ (nh_)
    {
        image_sub_ = it_.subscribe("/camera/image_raw",1,&ImageConverter::imageCb, this);
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



      this->imageBoardCapture(cv_ptr);
        cv::waitKey(3);

    }

    void imageBoardCapture(cv_bridge::CvImagePtr cv_ptr)
    {
        Image_ = Mat(cv_ptr->image);
        imshow("win1", Image_);
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imgRedTracker");
    ImageConverter ic;
    ros::spin();
return 0;
}
