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
    int lastX1_, lastY1_, lastX2_, lastY2_;
    int gaussianScalar_, edgeScalar_, edgeScalarUpper_, thresholdLowerScalar_, alpha_, beta_, gamma_;

    Mat imgLines;
    bool initFlag;

    Mat distCoeffs_, cameraMatrix_;

    double fx, fy, cx, cy, k1, k2, k3, p1, p2;

    Mat map1, map2;

    time_t timeLast_, timeNow_;
    int frames;
    double fps;

public:
    ImageConverter()
        : it_ (nh_)
    {
        image_sub_ = it_.subscribe("/camera/image_raw",1,&ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/banana_test/image_raw", 1);


        fx = 470.3510921455757;
        fy = 471.0176872702226;
        cx = 932.6785648556851;
        cy = 613.5156383470329;

        k1 = -0.2021686651096676;
        k2 = 0.02880744953416268;
        p1 = 0.0003215452567082094;
        p2 = 0.0008122247982410801;
        k3 = -0.001414257213967143;

        distCoeffs_ = Mat(1, 4, CV_64FC1);
        distCoeffs_.at<double>(0,0) = (k1);
        distCoeffs_.at<double>(0,1) = (k2);
        distCoeffs_.at<double>(0,2) = (p1);
        distCoeffs_.at<double>(0,3) = (p2);
        distCoeffs_.at<double>(0,4) = (k3);

        cameraMatrix_ = (Mat_<double>(3,3) << fx,0,cx,0,fy,cy,0,0,1);

        initFlag = true;

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

            if(initFlag)
            {
                initFlag = false;
                initUndistortRectifyMap(cameraMatrix_, distCoeffs_, Mat(),
                    //getOptimalNewCameraMatrix(cameraMatrix_, distCoeffs_, cv_ptr->image.size(), 1),
                    cameraMatrix_,
                    cv_ptr->image.size(), CV_32FC1, map1, map2);
                time(&timeLast_);

            }
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        this->imageUndistort(cv_ptr);
        this->FPSGet();
        int c = cv::waitKey(3);
        if((char)c == 27) initFlag = false;

    }

    void FPSGet()
    {
        time(&timeNow_);
        frames++;
        double seconds = difftime(timeNow_, timeLast_);
        if(seconds > 1.0)
        {
            time(&timeLast_);
            fps = frames/seconds;
            frames = 0;
            std::cout<<"FPS:"<<fps<<std::endl;
        }

        return;
    }

    void imageUndistort(cv_bridge::CvImagePtr cv_ptr)
    {
        Mat frame(cv_ptr->image);

        Mat undistorted;

        //undistort(frame, undistorted, cameraMatrix_, distCoeffs_);

        remap(frame, undistorted, map1, map2, INTER_LANCZOS4);
        //resize(undistorted, undistorted, Size(640, 480));

        //cv_ptr->image = undistorted;
        //image_pub_.publish(cv_ptr->toImageMsg());
        //imshow("Undistorted", undistorted);

        return;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fisheye_node");
    ImageConverter ic;
    ros::spin();
    return 0;
}
