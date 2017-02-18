#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <time.h>
#include "/home/googly/opencv/modules/calib3d/src/fisheye.cpp"
#include <iostream>
#include <fstream>

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
    Mat map1, map2;
    Matx33d K;
    Vec4d distCoeffs;
    bool initDewarp_;
public:
    ImageConverter()
        : it_ (nh_)
    {
        image_sub_ = it_.subscribe("/camera/image_raw",1,&ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/dewarp/image_raw",1);
        frames = 0;
        fps = 0;
        time(&timeLast_);

        initDewarp_ = true;
        Mat distCoeffs_(1, 4, CV_64FC1);
        distCoeffs_.at<double>(0,0) = (0.0312438);
        distCoeffs_.at<double>(0,1) = (0.0964677);
        distCoeffs_.at<double>(0,2) = (-0.0858381);
        distCoeffs_.at<double>(0,3) = (0.02103);

        distCoeffs = distCoeffs_;
        K = Matx33d(393.3629060425109, 0, 605.513689123417,
                    0, 395.5368909952134, 603.0372825006165,
                    0, 0, 1);
/*
        std::ifstream dewarpConfigFile;
        std::string fileLocation( (std::string(getenv("HOME")) + "/catkin_ws/src/opencvtest/include/opencvtest/dewarpConfig.txt") );
        dewarpConfigFile.open(fileLocation.c_str());
        if(dewarpConfigFile.is_open())
        {
            std::string line;
            char* lineChar;
            while(getline(dewarpConfigFile, line)){
                lineChar = (char*)line.c_str();
                line = strtok(lineChar, ";");
                if(line == "K"){
                        Mat Kmat(3, 3, CV_64FC1);;
                        for(int i = 0; i < 3; i++)
                        {
                            for(int j = 0; j < 3; j++)
                            {
                                line = strtok(NULL, ";");
                                Kmat.at<double>(i,j) = strtod(line.c_str(), 0);
                            }
                        }
                        K = Kmat;
                }
                else if( line == "distCoeffs")
                {
                    for(int i = 0; i < 2; i++)
                    {
                        line = strtok(NULL,";");
                        distCoeffs_.at<double>(0,i) = strtod(line.c_str(), 0);
                    }
                    distCoeffs = distCoeffs_;
                }
                else
                {
                    ROS_INFO("Unknown config input: %s", lineChar);
                }
            }
            ROS_INFO("Dewarp Configuration complete.");
        }
        else
        {
            ROS_INFO("Dewarp Configuration file could not be found/opened... Running defaults.");
        }
        */
        std::cout << "K:" << K << std::endl << "Dist" << distCoeffs << std::endl;

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
        cv_ptr->image = this->imageDewarp(this->imageCrop(cv_ptr));
        image_pub_.publish(cv_ptr->toImageMsg());

    }

    Mat imageCrop(cv_bridge::CvImagePtr cv_ptr)
    {
        Mat frame(cv_ptr->image);
        Rect roi(Point(350,0),Size(1200,1200));
        return frame(roi);
    }

    Mat imageDewarp(Mat Image)
    {
        if(initDewarp_)
        {
            initDewarp_ = false;
            fisheye::initUndistortRectifyMap(K, distCoeffs, vector<Vec3f>(),
                                             getOptimalNewCameraMatrix(Mat(K), Mat(distCoeffs), Image.size(), 1),
                                             Image.size(), CV_32FC1, map1, map2);
        }
        Mat imageUndistorted;
        remap(Image, imageUndistorted, map1, map2, INTER_LANCZOS4);
        imshow("original", Image);
        imshow("dewarp", imageUndistorted);
        waitKey(3);
        return imageUndistorted;
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
    ros::init(argc, argv, "dewarp_node");
    ImageConverter ic;
    ros::spin();
    return 0;
}
