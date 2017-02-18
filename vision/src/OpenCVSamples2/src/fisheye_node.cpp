#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    int lastX1_, lastY1_, lastX2_, lastY2_;
    int gaussianScalar_, edgeScalar_, edgeScalarUpper_, thresholdLowerScalar_, alpha_, beta_, gamma_;

    Mat imgLines;
    bool initFlag;

    Mat distCoeffs_, cameraMatrix_;

    int fx, fy, cx, cy, k1, k2, p1, p2;

public:
    ImageConverter()
        : it_ (nh_)
    {
        image_sub_ = it_.subscribe("/camera/image_raw",1,&ImageConverter::imageCb, this);

        namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
        gaussianScalar_ = 4000;

        fx = 215206090;
        fy = 254898493;
        cx = 315927657;
        cy = 239596656;
        k1 = 488288575;
        k2 = 1231009508;
        p1 = 1001275570;
        p2 = 1001518221;

        /*
        fx = 215.2060898295131;
        fy = 254.8984931247934;
        cx = 315.9276571237443;
        cy = 239.5966560069066;
        k1 = -0.48828857489206910;
        k2 = 0.23100950794183600;
        p1 = 0.00127557024969628;
        p2 = 0.00151822092630464;
*/
        //Create trackbars in "Control" window

        createTrackbar("fx", "Control", &fx, 500000000);
        createTrackbar("fy", "Control", &fy, 500000000);
        createTrackbar("cx", "Control", &cx, 500000000);
        createTrackbar("cy", "Control", &cy, 500000000);
        createTrackbar("k1", "Control", &k1, 2000000000);
        createTrackbar("k2", "Control", &k2, 2000000000);
        createTrackbar("p1", "Control", &p1, 2000000000);
        createTrackbar("p2", "Control", &p2, 2000000000);

        distCoeffs_ = Mat(1, 4, CV_64FC1);

        initFlag = true;
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

        this->imageUndistort(cv_ptr);

        int c = cv::waitKey(3);
        if((char)c == 27) initFlag = false;

        //image_pub_.publish(cv_ptr->toImageMsg());
    }


    void imageUndistort(cv_bridge::CvImagePtr cv_ptr)
    {
        Mat frame(cv_ptr->image);
        resize(frame, frame, Size(640, 480));
        Mat undistorted;

        distCoeffs_.at<double>(0,0) = (k1 -1000000000)/1000000000.0;
        distCoeffs_.at<double>(0,1) = (k2 -1000000000)/1000000000.0;
        distCoeffs_.at<double>(0,2) = (p1 -1000000000)/1000000000.0;
        distCoeffs_.at<double>(0,3) = (p2 -1000000000)/1000000000.0;

        Mat M = (Mat_<double>(3,3) << (fx/1000000.0),0,(cx/1000000.0),0,(fy/1000000.0),(cy/1000000.0),0,0,1);
        undistort(frame, undistorted, M, distCoeffs_);

        imshow("Undistorted", undistorted);

        return;
    }


};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imageConverter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
