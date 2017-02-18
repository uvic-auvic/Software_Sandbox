#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include "rov_camera/CameraControl.h"

using namespace cv;
RNG rng(12345);

class ImageConverter
{
    Mat imgLines, imgLinesLines;
    vector<Point> Last_;
    ros::NodeHandle nh_;
    ros::Subscriber gui_sub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    int lastX1_, lastY1_, lastX2_, lastY2_;
    int gaussianScalar_, edgeScalar_, edgeScalarUpper_, thresholdLowerScalar_, eps_;
    int maxAngle_, minAngle_, maxVertices_;
    int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV, iLastX, iLastY;

    Mat edgeLast_, imgFrame_, imgFrameOrig_;
    vector<Vec3f> circles_;
    vector<Point> center;
    vector<int> radius;



    bool initFlag;

public:

    ImageConverter()
        : it_ (nh_)
    {
        image_sub_ = it_.subscribe("/usb_cam/image_raw",1,&ImageConverter::imageCb, this);

        gaussianScalar_ = 5;
        thresholdLowerScalar_ = 100;
        edgeScalar_ = 100;
        edgeScalarUpper_ = 100;
        eps_ = 5;
        maxAngle_ = 180;
        minAngle_ = 0;
        maxVertices_ = 4;

        iLowH = 0;
        iHighH = 10;
        iLowS = 25;
        iHighS = 225;
        iLowV = 25;
        iHighV = 225;

        namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

        //Create trackbars in "Control" window
        createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
        createTrackbar("HighH", "Control", &iHighH, 179);

        createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
        createTrackbar("HighS", "Control", &iHighS, 255);

        createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
        createTrackbar("HighV", "Control", &iHighV, 255);

        for(int i = 0; i < Last_.size(); i++ )
            Last_[i] = Point(-1,-1);
        initFlag = true;

    }

    ~ImageConverter()
    {
        cv::destroyAllWindows();
    }

    void controlCb(rov_camera::CameraControl msg)
    {
        gaussianScalar_ = msg.gaussianScalar_;
        edgeScalar_ = msg.edgeScalar_;
        maxAngle_ = msg.maxAngle_;
        minAngle_ = msg.minAngle_;
        maxVertices_ = msg.maxVertices_;

    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            if(initFlag == true){
                imgLines = Mat::zeros(cv_ptr->image.size(),CV_8UC3);
                imgLinesLines = Mat::zeros(cv_ptr->image.size(),CV_8UC3);
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

    float angle( Point pt1, Point pt2, Point pt0)
    {
        float dx1 = pt1.x - pt0.x;
        float dy1 = pt1.y - pt0.y;
        float dx2 = pt2.x - pt0.x;
        float dy2 = pt2.y - pt0.y;
        float dot = dx1*dx2 + dy1*dy2;
        float det = dx1*dy2 - dx2*dy1;
        float result = atan2(det, dot);
        return result;
    }


    vector<vector<Point> > contoursApproxPoly(vector<vector<Point> > contours)
    {
        /* Function to find what shapes the contours make. User controlled to set up based on how many vertices the contours have
         * I.E. 4 vertices = a rectangular shape
         */
        vector<vector<Point> > result(contours.size());
        vector<Moments> mu(contours.size());
        vector<Point2f> mc(contours.size());
        vector<bool> shapeFlag;
        shapeFlag.resize(contours.size(), false);
        vector<bool> printed;
        printed.resize((circles_.size()), false);

        for (int i = 0; i < contours.size(); i++)
        {
            /* approxPolyDp is used to approximate contour enclosures */
            approxPolyDP(Mat(contours[i]), result[i], arcLength(Mat(contours[i]), true)*eps_/100.0, true);
            if((fabs(contourArea(result[i]))>1000) && isContourConvex(Mat(result[i])))
            {
                shapeFlag[i] = true;
            }
        }
        for(int i = 0; i < circles_.size(); i++)
        {
            bool printCircle = true;
            for(int j = i - 1; j >= 0; j--)
            {
                if(printed[j] == true)
                {
                    if((abs(norm(center[i] - center[j])) < 30) && (abs(radius[i] - radius[j]) < 20))
                    {
                        printCircle = false;
                        break;
                    }
                }
            }
            if(printCircle)
            {
                Scalar colour = Scalar(128,255,255);
                circle(imgLines, center[i], 3, colour, -1, 8, 0);
                circle(imgLines, center[i], radius[i], colour, 3, 8, 0);
                printed[i] = true;
            }
        }

        for (int i = 0; i < result.size(); i++)
        {
            if(shapeFlag[i])
            {
                /* if result[i].size() == maxvertices (i.e. 4, then it is a shape with 4 edges (rectangle)
                 * fabs(contourArea)>1000 arbitrary positive area. This removes negative area shapes and shapes too small to be considered important
                 * isContourConvex ensures the shape is convex and does not intersect itself
                 */
                float maxCosine = 0;
                float minCosine = M_PI;
                mu[i] = moments(result[i], false);
                mc[i] = Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);
                for(int j = 1; j < maxVertices_ + 1 ; j++)
                {
                    float cosine;
                    if (j == 1)
                        cosine = fabs(angle(result[i][result[i].size()-1], result[i][1], result[i][0]));
                    else
                        cosine = fabs(angle(result[i][j%maxVertices_], result[i][j-2], result[i][j-1]));
                    maxCosine = MAX(maxCosine, cosine);
                    minCosine = MIN(minCosine, cosine);
                }
                maxCosine = maxCosine *180/M_PI;
                minCosine = minCosine *180/M_PI;
                if(maxCosine < maxAngle_ && minCosine > minAngle_)
                {
                    /* User controlled angles. Allows refining such that shapes are excluded when its interior angles are exceeded. */
                    Scalar colour = Scalar(255,128,255);
                    polylines(imgLines, result[i], true, colour, 2);
                    circle(imgLines, mc[i], 4, colour, -1, 8, 0);
                }
            }
        }
        imgFrame_ = imgLines;
        return result;
    }



    void findCircles(Mat imgThresh)
    {
        return;
        HoughCircles(imgThresh, circles_, CV_HOUGH_GRADIENT, 2, 1 , 300, 150, 50);
        center.resize(circles_.size());
        radius.resize(circles_.size());


        for(int i = 0; i < circles_.size(); i++)
        {

            center[i] = Point(circles_[i][0], circles_[i][1]);
            radius[i] = circles_[i][2];
        }
        return;
    }

    void trackObject(Mat imgThresh)
    {
        Mat edges;
        vector<vector<Point> >contours;
        vector<Vec4i> hierarchy;
        /* Canny detects edges in the image. A user controlled edgeScalar is used to determine the edge thresholds */
        Canny(imgThresh, edges, edgeScalar_, edgeScalar_*2, 3, true);

        morphologyEx(edges, edges, MORPH_CLOSE, noArray(), Point(-1, -1), 1 );

        imshow("Edges", edges);

        /* Find the contours in the imgThresh image and store them in contours to be further processed */
        findContours(edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
        /* Find Circles in the image and draw them onto the image */
        findCircles(edges);
        /* approximate the contours into polynomials and draw them onto the image */
        vector<vector<Point> > approxPolyPoints = contoursApproxPoly(contours);


        return;
    }

    void imageEdgeTracker(cv_bridge::CvImagePtr cv_ptr)
    {
        Mat frame(cv_ptr->image);
        Mat imgHSV;
        Mat imgThresholded;


        imgLines = frame.clone();
        if(gaussianScalar_ > 0 )
        {
            /* Blur function in order to reduce noise. Guassian Blur will provide a better result at the cost of more processessing time.
             * Blur will provide faster processing time but worse results
             * Median Blur provides good results and fast processing time for small scalars.
             * Median blur scalar must be odd. Limited to 7 as values higher take high processessing time and 7 is already really blurry.
             */
            if(gaussianScalar_%2 == 0)
                gaussianScalar_ = gaussianScalar_ - 1;
            if(gaussianScalar_ > 7)
                gaussianScalar_ = 7;
            medianBlur(frame,frame, gaussianScalar_);
        }


        /* convert the blurred image to gray as this is easier to detect edges in */
        cvtColor(frame, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to GRAY
        inRange(imgHSV, Scalar(iLowH,iLowS,iLowV), Scalar(iHighH,iHighS,iHighV), imgThresholded);

        /* erode/dilate to remove small holes from the foreground*/
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)), Point(-1, -1), 2);
        dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)), Point(-1, -1), 2);


        if(initFlag)
        {
            initFlag = false;
        }

        trackObject(imgThresholded);
        imshow("imgFrame", imgFrame_);
        return;
    }


};


int main(int argc, char** argv) {
    ros::init(argc, argv, "imageConverter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
