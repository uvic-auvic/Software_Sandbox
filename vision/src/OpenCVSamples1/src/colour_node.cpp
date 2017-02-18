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

    int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV, iLastX, iLastY;

public:
    ImageConverter()
        : it_ (nh_)
    {
        image_sub_ = it_.subscribe("/usb_cam/image_raw",1,&ImageConverter::imageCb, this);
        frames = 0;
        fps = 0;
        time(&timeLast_);

        iLowH = 170;
        iHighH = 179;
        iLowS = 150;
        iHighS = 255;
        iLowV = 60;
        iHighV = 255;
        namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

        //Create trackbars in "Control" window
        createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
        createTrackbar("HighH", "Control", &iHighH, 179);

        createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
        createTrackbar("HighS", "Control", &iHighS, 255);

        createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
        createTrackbar("HighV", "Control", &iHighV, 255);
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
        this->imgThreshold(cv_ptr);
    }

    void imgThreshold(cv_bridge::CvImagePtr cv_ptr)
    {
        Mat frame(cv_ptr->image);
        Mat imgThresholded, HSV, LowerBound, UpperBound;

        if(iLowH <= iHighH)
        {
            /* if iLowH <= iHighH assume normal inRange operation */
            cvtColor(frame, HSV, CV_BGR2HSV);
            inRange(HSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV),imgThresholded);

            /* Display Lower and Upper bounds for the scalar ranges in control */
            /* Create Lower and Upper bound Matrices from the HSV Scalar and convert to RGB colourspace (imshow doesn't handle HSV well) */
            cvtColor(Mat(100,100, CV_8UC3).setTo(Scalar(iLowH, iLowS, iLowV)), LowerBound, CV_HSV2BGR);
            cvtColor(Mat(100,100, CV_8UC3).setTo(Scalar(iHighH, iHighS, iHighV)), UpperBound, CV_HSV2BGR);
        }
        else if(iLowH > iHighH)
        {
            /* if iLowH > iHighH then assume the user wants to wrap around the colour wheel so shift the colour wheel
             * in order for the wheel to wrap around red. */
            cvtColor(frame, HSV, CV_RGB2HSV);
            inRange(HSV, Scalar(iHighH, iLowS, iLowV), Scalar(iLowH, iHighS, iHighV),imgThresholded);

            /* Display Lower and Upper bounds for the scalar ranges in control */
            /* Create Lower and Upper bound Matrices from the HSV Scalar and convert to RGB colourspace (imshow doesn't handle HSV well) */
            cvtColor(Mat(100,100, CV_8UC3).setTo(Scalar(iHighH, iLowS, iLowV)), LowerBound, CV_HSV2RGB);
            cvtColor(Mat(100,100, CV_8UC3).setTo(Scalar(iLowH, iHighS, iHighV)), UpperBound, CV_HSV2RGB);
        }



        /* Create Mat combine with space to hold both Upper and Lower bound matrices */
        Mat combine(max(LowerBound.size().height, UpperBound.size().height), LowerBound.size().width+UpperBound.size().width, CV_8UC3);
        /* Fill left half of combine with Lower bound mat */
        Mat left_roi(combine, Rect(0,0,LowerBound.size().width, LowerBound.size().height));
        LowerBound.copyTo(left_roi);
        /* Fill right half of combine with Upper bound mat */
        Mat right_roi(combine, Rect(LowerBound.size().width, 0, UpperBound.size().width, UpperBound.size().height));
        UpperBound.copyTo(right_roi);
        imshow("Control", combine);

        /* Create a copy of the original, only showing the colours desired) */
        Mat maskedOriginal;
        frame.copyTo(maskedOriginal, imgThresholded);

        /* Display images/frames */
        imshow("Original",frame);
        imshow("Threshold", imgThresholded);
        imshow("Masked Original", maskedOriginal);

        /* Need to call waitKey for imshow to work */
        waitKey(3);
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
    ros::init(argc, argv, "orientation_node");
    ImageConverter ic;
    ros::spin();
    return 0;
}
