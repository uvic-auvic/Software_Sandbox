#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <math.h>
#include <ctime>

#define VIDEO_FILE_LOCATION "/ROV02/video/"
using namespace cv;

/* Some starting scalars to use for the colour function call */
/* this will need to be set dependant on the lighting environment */
/* Perhaps they should be able to be changed in the GUI or at the least changed in a config file */
#define RED_UPPER Scalar(133,255,255)
#define RED_LOWER Scalar(112,41,41)
#define GREEN_UPPER Scalar(86,255,255)
#define GREEN_LOWER Scalar(44,25,27)
#define WHITE_UPPER Scalar(179,34,255)
#define WHITE_LOWER Scalar(0,0,75)
#define ORANGE_UPPER Scalar(111,255,255)
#define ORANGE_LOWER Scalar(93,41,39)


class ImageConverter
{
    ros::NodeHandle nh_;
    ros::Subscriber gui_sub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    CascadeClassifier buoy_classifier;

    cv::Size imageSize;

    bool initFlag;

public:

    ImageConverter()
        : it_ (nh_)
    {
        image_sub_ = it_.subscribe("/usb_cam/image_raw",1,&ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/GUI/camera", 1);

        initFlag = true;

        if(!buoy_classifier.load( "/home/googly/catkin_ws/src/opencvtest/include/opencvtest/buoy_cascade.xml"))
        {
            std::cout << "Error loading buoy cascade." << std::endl;
        }
        ROS_INFO("Configuration Complete.");
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
                imageSize = cv_ptr->image.size();
                initFlag = false;
            }

        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        /* Do stuff here. */
        imshow("original", cv_ptr->image); //Example, show the original image/frame
        imshow("cascade", cascade(buoy_classifier, 1.1, 3, cv_ptr->image)); //Example, apply a cascade classifier and find buoy object
        imshow("colour", colorThreshold(cv_ptr->image, RED_LOWER, RED_UPPER)); //Example, find colour red and show it
        Mat orientationImage = cv_ptr->image.clone();
        getContours(orientationImage, colorThreshold(cv_ptr->image, RED_LOWER, RED_UPPER));
        imshow("orientation", orientationImage);

        cv::waitKey(1);

        image_pub_.publish(cv_ptr->toImageMsg());
    }


    Mat colorThreshold(Mat image2threshold, Scalar Lower, Scalar Upper)
    {
        /* takes in a BGR color image and returns a thresholded image with input colors thresholded */
        Mat imageThresholded;
        cvtColor(image2threshold, imageThresholded, CV_RGB2HSV);
        medianBlur(imageThresholded, imageThresholded, 3);
        inRange(imageThresholded, Lower, Upper, imageThresholded);
        return imageThresholded;
    }

    Mat cascade (CascadeClassifier classifier, double scaleFactor, int minNeighbours, Mat img2Thresh)
    {
        /* Takes in an image/frame and a classifier with parameters and applies the cascade classifier to search for the object. */
        /* Returns a copy of the original image with an ellipse around all the found objects */
        /* This function should be changed to either directly output the center coordinates of the object to the TF world, or to return
         * the center points for them to be sent to the TF world in a later function */
        Mat imgThresholded;
        Mat imgFrame;

        imgFrame = img2Thresh.clone();

        cvtColor(img2Thresh, imgThresholded, CV_BGR2GRAY);
        equalizeHist( imgThresholded, imgThresholded);

        /* Apply Cascade classifier to search for object */
        vector<Rect> objects;
        if(scaleFactor < 1.05)
            scaleFactor = 1.05;

        classifier.detectMultiScale(imgThresholded, objects, scaleFactor, minNeighbours,0|CV_HAAR_SCALE_IMAGE, Size(5,5), Size(imgThresholded.cols/2, imgThresholded.rows/2));
        for(int i = 0; i < objects.size(); i++)
        {
            Point center( (objects[i].x + objects[i].width*0.5), (objects[i].y + objects[i].height*0.5) );
            ellipse ( imgFrame, center, Size(objects[i].height*0.5, objects[i].width*0.5), 0, 0, 360, Scalar(255,128,255), 4, 8, 0);
        }

        return imgFrame;
    }

    void getContours(Mat &src, Mat imgThresholded)
    {
        vector<Vec4i> hierarchy;
        vector<vector<Point> > contours;
        findContours(imgThresholded, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
        for(size_t i = 0; i < contours.size(); i++)
        {
            double area = contourArea(contours[i]);
            if(area > 1e2 && 1e5 > area)
            {
                drawContours(src, contours, static_cast<int>(i), Scalar(0,0,255),2,8,hierarchy, 0);
                approxPolyDP(Mat(contours[i]), contours[i], arcLength(Mat(contours[i]), true)*0.02, true);
                getOrientation(contours[i], src);
            }
        }
        return;
    }

    double getOrientation(vector<Point> &pts, Mat &img)
    {
        //Construct a buffer used by the pca analysis
        Mat data_pts = Mat(pts.size(), 2, CV_64FC1);
        for (int i = 0; i < data_pts.rows; ++i)
        {
            data_pts.at<double>(i, 0) = pts[i].x;
            data_pts.at<double>(i, 1) = pts[i].y;
        }

        //Perform PCA analysis
        PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);

        //Store the position of the object
        Point pos = Point(pca_analysis.mean.at<double>(0, 0),
                          pca_analysis.mean.at<double>(0, 1));

        //Store the eigenvalues and eigenvectors
        vector<Point2d> eigen_vecs(2);
        vector<double> eigen_val(2);
        for (int i = 0; i < 2; ++i)
        {
            eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                    pca_analysis.eigenvectors.at<double>(i, 1));

            eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
        }

        // Draw the principal components
        circle(img, pos, 3, CV_RGB(255, 0, 255), 2);
        line(img, pos, pos + 0.02 * Point(eigen_vecs[0].x * eigen_val[0], eigen_vecs[0].y * eigen_val[0]) , CV_RGB(255, 255, 0));
        line(img, pos, pos + 0.02 * Point(eigen_vecs[1].x * eigen_val[1], eigen_vecs[1].y * eigen_val[1]) , CV_RGB(0, 255, 255));

        return atan2(eigen_vecs[0].y, eigen_vecs[0].x);
    }


};


int main(int argc, char** argv) {
    ros::init(argc, argv, "auv_cam");
    ImageConverter ic;
    ros::spin();
    return 0;
}
