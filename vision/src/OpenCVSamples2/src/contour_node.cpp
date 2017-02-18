#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
RNG rng(12345);

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


public:
    ImageConverter()
        : it_ (nh_)
    {
        image_sub_ = it_.subscribe("/usb_cam/image_raw",1,&ImageConverter::imageCb, this);

        namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
        gaussianScalar_ = 4000;
        thresholdLowerScalar_ = 100;
        edgeScalar_ = 50;
        edgeScalarUpper_ = 100;
        alpha_ = 5;
        gamma_ = 0;
        //Create trackbars in "Control" window
        createTrackbar("Gaussian Filter", "Control", &gaussianScalar_, 4000);
        createTrackbar("Upper Edge Threshold", "Control", &edgeScalarUpper_, 255);
        createTrackbar("Lower Threshold", "Control", &edgeScalar_, 255);
        createTrackbar("Dark Threshold", "Control", &thresholdLowerScalar_, 255);
        createTrackbar("Alpha", "Control", &thresholdLowerScalar_, 10);
        createTrackbar("Gamma", "Control", &gamma_, 10);

        cvNamedWindow("Original");
        cvNamedWindow("Threshold");
        cvNamedWindow("Edges");

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
            if(initFlag == true){
                imgLines = Mat::zeros(cv_ptr->image.size(),CV_8UC3);
                initFlag = false;
            }
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        this->imageEdgeTracker(cv_ptr);

        int c = cv::waitKey(3);
        if((char)c == 27) ros::shutdown();

        image_pub_.publish(cv_ptr->toImageMsg());
    }

    void trackObject(Mat imgThresh)
    {
        vector<vector<Point> >contours;
        vector<Vec4i> hierarchy;

        findContours(imgThresh, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

        for( int i = 0; i < contours.size(); i++)
        {
            Scalar color = Scalar( rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
            drawContours( imgLines, contours, i, color, 2, 8, hierarchy, 0, Point() );
        }

        return;
    }

    void imageEdgeTracker(cv_bridge::CvImagePtr cv_ptr)
    {
        Mat frame(cv_ptr->image);

        beta_ = 10 - alpha_;
        Mat imgThresholded;
        Mat edges;
        Mat blended;

        if(gaussianScalar_ <= 0 )
            gaussianScalar_ = 1;
        cvtColor(frame, imgThresholded, COLOR_BGR2GRAY); //Convert the captured frame from BGR to GRAY
        imgLines = frame.clone();
        GaussianBlur(imgThresholded, imgThresholded, cv::Size(0,0), gaussianScalar_/1000.0, gaussianScalar_/1000.0);

        Canny(imgThresholded, edges, edgeScalar_, edgeScalarUpper_);
        //cvtColor(edges, blended, COLOR_GRAY2BGR);
        trackObject(edges);

        //imshow("Original", frame);
        //imshow("Threshold", imgThresholded);
        //imshow("Edges", edges);

        imshow("Edge Image", imgLines);

        return;
    }


};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imageConverter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
