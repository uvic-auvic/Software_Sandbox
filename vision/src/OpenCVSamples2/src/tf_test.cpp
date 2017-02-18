#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include "rov_camera/CameraControl.h"
#include <tf/transform_broadcaster.h>

using namespace cv;
RNG rng(12345);

class ImageConverter
{
    Mat imgLines, imgLinesLines;

    ros::NodeHandle nh_;
    ros::Subscriber gui_sub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;


    int gaussianScalar_, edgeScalar_, edgeScalarUpper_, thresholdLowerScalar_, eps_;
    int maxAngle_, minAngle_, maxVertices_;

    Mat edgeLast_, imgFrame_;

    bool initFlag;

public:

    ImageConverter()
        : it_ (nh_)
    {
        image_sub_ = it_.subscribe("/usb_cam/image_raw",1,&ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/GUI/camera", 1);
        gui_sub_ = nh_.subscribe("/GUI/camera_controls", 1, &ImageConverter::controlCb, this);

        gaussianScalar_ = 5;
        thresholdLowerScalar_ = 100;
        edgeScalar_ = 100;
        edgeScalarUpper_ = 100;
        eps_ = 5;
        maxAngle_ = 100;
        minAngle_ = 80;
        maxVertices_ = 4;
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

        for (int i = 0; i < contours.size(); i++)
        {
            /* approxPolyDp is used to approximate contour enclosures */
            approxPolyDP(Mat(contours[i]), result[i], arcLength(Mat(contours[i]), true)*eps_/100.0, true);
            if((fabs(contourArea(result[i]))>1000) && isContourConvex(Mat(result[i])))
            {
                shapeFlag[i] = true;
            }
        }

        for (int i = 0; i < result.size(); i++)
        {
             if((result[i].size() == maxVertices_) && shapeFlag[i])
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
                    Scalar colour = ObjectColour(result,shapeFlag, i);
                    polylines(imgLines, result[i], true, colour, 2);
                    circle(imgLines, mc[i], 4, colour, -1, 8, 0);
                    sendTF(mc[i], "square");
                }
            }
        }
        imgFrame_ = imgLines;
        return result;
    }



    Scalar ObjectColour(vector<vector<Point> > contour, vector<bool> shapeFlag, int i)
    {
        /* Function creates a mask around the contour region so that only the pixels inside the contour are visible
         * Then uses a mean to get the average colour inside the contour region. This is used for determining the colour
         * inside the contour shape.
         */
        Rect roi = boundingRect(contour[i]); // bounding rect to reduce the image size to only include region of interest.

        Mat mask(imgLines.size(), CV_8UC1, Scalar(0)); // mask for the exact shape of the contour, this is to extract only the pixels of interest.

        /* Fill the mask for the contour shape */
        const Point* elementPoints[1] = { &contour[i][0] };
        int npts = contour[i].size();
        fillPoly( mask, elementPoints, &npts, 1, Scalar(255),8);

        /* Check if there are other shapes inside this shape. This will eliminate them from contributing towards the colour of the shape */
        for(int j = 0; j < contour.size(); j++)
        {
            if(j != i && shapeFlag[j])
            {
                int shapeCheck = 0;
                for(int k = 0; k < contour[j].size(); k++)
                {
                    if(pointPolygonTest(contour[i], contour[j][k], true) > 0)
                    {
                        /* If true, polygon point is inside this contour. */
                        shapeCheck++;
                    }
                }
                if(shapeCheck >= contour[j].size())
                {
                    /* All points of the contour[j] shape is inside the other main contour[i]. This shape should not be considered for the object and is taken out. */
                    const Point* elementPoints[1] = { &contour[j][0] };
                    int npts = contour[j].size();
                    fillPoly( mask, elementPoints, &npts, 1, Scalar(0),8);
                }
            }
        }

        Mat imgROI(imgLines.size(), CV_8UC3);
        imgLines.copyTo(imgROI, mask);

        /* Cut the image down to only the bounded rectangle region of interest */
        Mat contourReg = imgROI(roi);
        Mat meanMask = mask(roi);
        /* Get the mean colour for the contour excluding contours inside this shape */
        Scalar colour = mean(contourReg, meanMask);

        return colour;
    }

    void trackObject(Mat imgThresh)
    {
        Mat edges;
        vector<vector<Point> >contours;
        vector<Vec4i> hierarchy;
        /* Canny detects edges in the image. A user controlled edgeScalar is used to determine the edge thresholds */
        Canny(imgThresh, edges, edgeScalar_, edgeScalar_*2, 3, true);

        morphologyEx(edges, edges, MORPH_CLOSE, noArray(), Point(-1, -1), 1 );
        /* Find the contours in the imgThresh image and store them in contours to be further processed */
        findContours(edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
        /* approximate the contours into polynomials and draw them onto the image */
        vector<vector<Point> > approxPolyPoints = contoursApproxPoly(contours);

        return;
    }

    void imageEdgeTracker(cv_bridge::CvImagePtr cv_ptr)
    {
        Mat frame(cv_ptr->image);
        Mat imgThresholded;

        imgLines = frame.clone();
        if(gaussianScalar_ > 0 )
        {
            /* Blur function in order to reduce noise. Guassian Blur will provide a better result at the cost of more processessing time.
             * Blur will provide faster processing time but worse results
             * Median Blur provides good results and fast processing time for small scalars.
             * Median blur scalar must be odd. Limited to 7 as values higher take high processessing time and 7 is already really blurry.
             */
            //GaussianBlur(frame, frame, cv::Size(0,0), gaussianScalar_/1000.0, gaussianScalar_/1000.0);
            if(gaussianScalar_%2 == 0)
                gaussianScalar_ = gaussianScalar_ - 1;
            if(gaussianScalar_ > 7)
                gaussianScalar_ = 7;
            medianBlur(frame,frame, gaussianScalar_);
            //bilateralFilter(frame, frame, 5, gaussianScalar_/100.0, gaussianScalar_/100.0);
            //blur(frame, frame, Size((trunc(gaussianScalar_/1000.0)),(trunc(gaussianScalar_/1000.0))), Point(-1,-1));
        }

        /* convert the blurred image to gray as this is easier to detect edges in */
        cvtColor(frame, imgThresholded, COLOR_BGR2GRAY); //Convert the captured frame from BGR to GRAY

        /* erode/dilate to remove small holes from the foreground*/
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)), Point(-1, -1));
        dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)), Point(-1, -1));

        if(initFlag)
        {
            initFlag = false;
        }

        trackObject(imgThresholded);
        /* Republish the video stream so that it can be viewed in the gui */
        cv_ptr->image = imgFrame_;
        image_pub_.publish(cv_ptr->toImageMsg());
        return;
    }

    void sendTF(Point2f center, std::string shape_name)
    {
        tf::TransformBroadcaster tf_;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(center.x,center.y,0.0));
        tf_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", shape_name));
        return;
    }

};


int main(int argc, char** argv) {
    ros::init(argc, argv, "imageConverter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
