#include <cv.h>
#include <highgui.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "/home/googly/opencv/modules/calib3d/src/fisheye.cpp"
#include <opencv2/core/version.hpp>

using namespace cv;

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    int numCornersHor, numBoards, numCornersVer, numSquares;
    int successes;
    Size board_sz;

    vector<vector<Point3f> > object_points;
    vector<vector<Point2f> > image_points;
    vector<Point2f> corners;

    Mat intrinsic;
    Matx33d K;
    Vec4d distCoeffs;
    Mat map1, map2;

    vector<Point3f> obj;
    bool printFlag_;
public:
    ImageConverter()
        : it_ (nh_)
    {
        image_sub_ = it_.subscribe("/crop/image_raw",1,&ImageConverter::imageCb, this);
        numBoards = 0;
        std::cout << "CV Version: "<<CV_VERSION <<std::endl;
        printf("Enter number of corners along width: ");
        scanf("%d", &numCornersHor);

       printf("Enter number of corners along height: ");
       scanf("%d", &numCornersVer);

       printf("Enter number of boards: ");
       scanf("%d", &numBoards);

       numSquares = numCornersHor * numCornersVer;
       board_sz = Size(numCornersHor, numCornersVer);

        successes=0;
        printFlag_ = true;
        intrinsic = Mat(3, 3, CV_32FC1);
        intrinsic.ptr<float>(0)[0] = 1;
        intrinsic.ptr<float>(1)[1] = 1;

        for(int j=0;j<numSquares;j++)
            obj.push_back(Point3f(j/numCornersHor, j%numCornersHor, 0.0f));

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
        Mat Image = Mat(cv_ptr->image);
        Mat gray_image;
        if(successes<numBoards)
        {
            cvtColor(Image, gray_image, CV_BGR2GRAY);
            bool found = findChessboardCorners(Image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

               if(found)
               {
                   cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
                   drawChessboardCorners(gray_image, board_sz, corners, found);
               }

               imshow("win1", gray_image);

               int key = waitKey(1);
               if(found!=0)
               {
                   image_points.push_back(corners);
                   object_points.push_back(obj);

                   std::cout<< "Snap count: " << successes << std::endl;

                   successes++;
               }
        }
        else
        {
            if(printFlag_)
            {
                K = intrinsic;
                vector<Vec3d> rvecs;
                vector<Vec3d> tvecs;
                fisheye::calibrate(object_points, image_points, Image.size(), K, distCoeffs, rvecs, tvecs, fisheye::CALIB_CHECK_COND +
                        fisheye::CALIB_FIX_SKEW + fisheye::CALIB_RECOMPUTE_EXTRINSIC);
                std::cout << "Intrinsic: " << K << std::endl << "DistCoeffs: " << distCoeffs << std::endl;
                printFlag_ = false;
                /*fisheye::initUndistortRectifyMap(K, distCoeffs, vector<Vec3f>(),
                                                 getOptimalNewCameraMatrix(Mat(K), Mat(distCoeffs), Image.size(), 1),
                                                 Image.size(), CV_32FC1, map1, map2);*/
            }
            Mat imageUndistorted;
            //remap(Image, imageUndistorted, map1, map2, INTER_LANCZOS4);
            //imshow("winUndistort", imageUndistorted);
            waitKey(1);

        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imgRedTracker");
    ImageConverter ic;

    ros::spin();
return 0;
}
