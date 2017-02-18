#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

using namespace cv;

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV, iLastX, iLastY;
    Mat imgLines;
    bool initFlag;

public:
    ImageConverter()
        : it_ (nh_)
    {
        image_sub_ = it_.subscribe("/usb_cam/image_raw",1,&ImageConverter::imageCb, this);

        //cv::namedWindow(OPENCV_WINDOW);
        iLowH = 170;
        iHighH = 179;
        iLowS = 150;
        iHighS = 255;
        iLowV = 60;
        iHighV = 255;
        iLastX = -1;
        iLastY = -1;

        namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

        //Create trackbars in "Control" window
        createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
        createTrackbar("HighH", "Control", &iHighH, 179);

        createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
        createTrackbar("HighS", "Control", &iHighS, 255);

        createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
        createTrackbar("HighV", "Control", &iHighV, 255);
        initFlag = true;
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
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

        this->imageRedTracker(cv_ptr);

        //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);

        image_pub_.publish(cv_ptr->toImageMsg());
    }

    void imageRedTracker(cv_bridge::CvImagePtr cv_ptr)
    {
        Mat imgOriginal(cv_ptr->image);


        Mat imgHSV;

         cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

         Mat imgThresholded;
         inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

          //morphological opening (removes small objects from the foreground)
          erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
          dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

          //morphological closing (removes small holes from the foreground)
          dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
          erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

          //Calculate the moments of the thresholded image
          Moments oMoments = moments(imgThresholded);

          double dM01 = oMoments.m01;
          double dM10 = oMoments.m10;
          double dArea = oMoments.m00;

          // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero
          if (dArea > 10000)
          {
           //calculate the position of the ball
           int posX = dM10 / dArea;
           int posY = dM01 / dArea;

           if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
           {
            //Draw a red line from the previous point to the current point
            line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0,0,255), 2);
           }

           iLastX = posX;
           iLastY = posY;
          }

          imshow("Thresholded Image", imgThresholded); //show the thresholded image

          imgOriginal = imgOriginal + imgLines;
          imshow("Original", imgOriginal); //show the original image
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imageConverter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
