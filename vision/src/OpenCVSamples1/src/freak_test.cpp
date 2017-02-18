#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/nonfree.hpp>
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

    Ptr<FeatureDetector> detector_;
    ORB extractor_;

    BruteForceMatcher<Hamming> matcher;
    Mat descriptorsA;
    Mat imgA;
    vector<KeyPoint> keypointsA;
public:
    ImageConverter()
        : it_ (nh_)
    {
    /* ImageConverter type initializer. Initializes all the class variables and the ros subscribers/publishers. */

        image_sub_ = it_.subscribe("/usb_cam/image_raw",1,&ImageConverter::imageCb, this); // subscribe to your camera
        image_pub_ = it_.advertise("/output/image_raw",1); // publish to your desired image topic
        frames = 0;
        time(&timeLast_);

        detector_ = new ORB();
        imgA = imread((std::string(getenv("HOME")) + "/catkin_ws/src/opencvtest/include/opencvtest/obj.png"), CV_LOAD_IMAGE_GRAYSCALE );
        if( !imgA.data ) {
            std::cout<< " --(!) Error reading image " << (std::string(getenv("HOME")) + "/catkin_ws/src/opencvtest/include/opencvtest/buoy.jpg") << std::endl;
            return;
        }
        medianBlur(imgA, imgA, 5);
        detector_->detect(imgA, keypointsA);
        extractor_.compute(imgA, keypointsA, descriptorsA);
    }

    ~ImageConverter()
    {
        cv::destroyAllWindows();
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
    /* ROS CallBack. Called when the image_pub_ has found a new message (new camera frame) */
        cv_bridge::CvImagePtr cv_ptr;
        /* Try to retrieve the new frame, if not, report the error */
        try
        {

            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        /* New frame has been obtained and can send the frame to different functions here for processing in OpenCV */
        this->FPSGet(); // gets the frame rate for this node
        this->getFreaky(cv_ptr);
    }

    void FPSGet()
    {
    /* Gets the FPS of the function. If the time is >= than 1 second, calculate how many frames have past in this time.
     * difftime returns how many seconds have past ( resolution is of seconds )
     * difftime's resolution will contribute to a small error in fps; however, this is a negligible amount of error and can
     * be dismissed.
     * Certain OpenCV functions will take up a large amount of processing. This will show up in the FPS, as the FPS will
     * lower. */
        time(&timeNow_);
        frames++;
        double seconds = difftime(timeNow_, timeLast_);
        if(seconds >= 1.0)
        {
            std::cout<<"FPS:"<<frames/seconds<<std::endl;
            time(&timeLast_);
            frames = 0;
        }
        return;
    }

    void getFreaky(cv_bridge::CvImagePtr cv_ptr)
    {
        /* Do your function things here. The function currently converts the cv_ptr into a Mat and shows it
         * in the window "Original" and then republish the frame. */
        Mat frame(cv_ptr->image);
        //frame = imread((std::string(getenv("HOME")) + "/catkin_ws/src/opencvtest/include/opencvtest/box_in_scene.png"), CV_LOAD_IMAGE_COLOR );

        Mat imgMatch, imgGray;
        vector<vector<DMatch> > matches;
        vector<KeyPoint> keypointsScene;
        Mat descriptorsScene;
        cvtColor(frame, imgGray, CV_BGR2GRAY);
        medianBlur(imgGray, imgGray, 3);
        detector_->detect(imgGray, keypointsScene);
        extractor_.compute(imgGray, keypointsScene, descriptorsScene);

        imshow("original", frame);
        if(descriptorsScene.size().height > 0)
        {
            matcher.knnMatch(descriptorsA, descriptorsScene, matches, 2);
            std::vector< DMatch > good_matches;

            for( int i = 0; i < matches.size(); ++i )
            {
                if( matches[i][0].distance < 0.6*matches[i][1].distance )
                {
                    good_matches.push_back( matches[i][0]);
                }
            }

            drawMatches(imgA, keypointsA, frame, keypointsScene, good_matches, imgMatch,Scalar(0,0,255), Scalar(75,75,75),
                        vector<char>());

            //-- Localize the object
            std::vector<Point2f> obj;
            std::vector<Point2f> scene;

            for( int i = 0; i < good_matches.size(); i++ )
            {
              //-- Get the keypoints from the good matches
              obj.push_back( keypointsA[ good_matches[i].queryIdx ].pt );
              scene.push_back( keypointsScene[ good_matches[i].trainIdx ].pt );
            }

            if(good_matches.size() > 4 )
            {
                Mat H = findHomography( obj, scene);

                //-- Get the corners from the image_1 ( the object to be "detected" )
                std::vector<Point2f> obj_corners(4);
                obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( imgA.cols, 0 );
                obj_corners[2] = cvPoint( imgA.cols, imgA.rows ); obj_corners[3] = cvPoint( 0, imgA.rows );
                std::vector<Point2f> scene_corners(4);

                perspectiveTransform( obj_corners, scene_corners, H);

                //-- Draw lines between the corners (the mapped object in the scene - image_2 )
                line( imgMatch, scene_corners[0] + Point2f( imgA.cols, 0), scene_corners[1] + Point2f( imgA.cols, 0), Scalar( 0, 255, 0), 4 );
                line( imgMatch, scene_corners[1] + Point2f( imgA.cols, 0), scene_corners[2] + Point2f( imgA.cols, 0), Scalar( 0, 255, 0), 4 );
                line( imgMatch, scene_corners[2] + Point2f( imgA.cols, 0), scene_corners[3] + Point2f( imgA.cols, 0), Scalar( 0, 255, 0), 4 );
                line( imgMatch, scene_corners[3] + Point2f( imgA.cols, 0), scene_corners[0] + Point2f( imgA.cols, 0), Scalar( 0, 255, 0), 4 );

            }
            imshow("matches", imgMatch);
        }
        waitKey(1);
        return;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "opencv_node");
    ImageConverter ic;
    ros::spin();
    return 0;
}
