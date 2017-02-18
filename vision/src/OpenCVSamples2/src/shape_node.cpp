#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>

using namespace cv;
RNG rng(12345);

Mat imgLines, imgLinesLines;
vector<Point> Last_;

void onTrackbar( int, void* )
{
    imgLinesLines = Mat::zeros(imgLines.size(),CV_8UC3);
    for(int i = 0; i < Last_.size(); i++)
        Last_[i] = Point(-1,-1);
}

class ImageConverter
{

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    int lastX1_, lastY1_, lastX2_, lastY2_;
    int gaussianScalar_, edgeScalar_, edgeScalarUpper_, thresholdLowerScalar_, eps_;
    int maxAngle_, minAngle_, maxVertices_;

    Mat edgeLast_, imgFrame_;
    bool initFlag;

public:

    ImageConverter()
        : it_ (nh_)
    {
        image_sub_ = it_.subscribe("/usb_cam/image_raw",1,&ImageConverter::imageCb, this);

        namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
        gaussianScalar_ = 1000;
        thresholdLowerScalar_ = 100;
        edgeScalar_ = 100;
        edgeScalarUpper_ = 100;
        eps_ = 5;
        maxAngle_ = 100;
        minAngle_ = 80;
        maxVertices_ = 4;
        //Create trackbars in "Control" window
        createTrackbar("Gaussian Filter", "Control", &gaussianScalar_, 4000, onTrackbar);
       //createTrackbar("Upper Edge Threshold", "Control", &edgeScalarUpper_, 255);
        createTrackbar("Edge Threshold", "Control", &edgeScalar_, 255, onTrackbar);
        //createTrackbar("Dark Threshold", "Control", &thresholdLowerScalar_, 255);
        createTrackbar("Epsilon", "Control", &eps_, 100, onTrackbar);
        createTrackbar("Max Angle", "Control", &maxAngle_, 180, onTrackbar);
        createTrackbar("Min Angle", "Control", &minAngle_, 180, onTrackbar);
        createTrackbar("Number of edges", "Control", &maxVertices_, 20, onTrackbar);
        for(int i = 0; i < Last_.size(); i++ )
            Last_[i] = Point(-1,-1);
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
        //return (dx1*dx2 + dy1 * dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
        float dot = dx1*dx2 + dy1*dy2;
        float det = dx1*dy2 - dx2*dy1;
        float result = atan2(det, dot);
        return result;
    }


    vector<vector<Point> > contoursConvexHull(vector<vector<Point> > contours)
    {
        vector<vector<Point> > result(contours.size());
        vector<Moments> mu(contours.size());
        vector<Point2f> mc(contours.size());

        for (int i = 0; i < contours.size(); i++)
        {
            convexHull(Mat(contours[i]), result[i], true);
            if(fabs(contourArea(result[i]))>100 && isContourConvex(Mat(result[i])))
            {
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
                    Scalar colour = Scalar(rng.uniform(0, 255),rng.uniform(0, 255),rng.uniform(0, 255));
                    polylines(imgLines, result[i], true, colour, 2);
                    circle(imgLines, mc[i], 4, colour, -1, 8, 0);
                }
            }
        }
        return result;
    }

    void ResizeLast(int newSize)
    {
        int oldSize = Last_.size();
        Last_.resize(newSize);
        for(int i = oldSize; i < newSize; i++ )
            Last_[i] = Point(-1,-1);
    }

    vector<vector<Point> > contoursApproxPoly(vector<vector<Point> > contours)
    {
        vector<vector<Point> > result(contours.size());
        vector<Moments> mu(contours.size());
        vector<Point2f> mc(contours.size());
        if(Last_.size() < contours.size())
        {
           ResizeLast(contours.size()+1);
        }

        for (int i = 0; i < contours.size(); i++)
        {
            approxPolyDP(Mat(contours[i]), result[i], arcLength(Mat(contours[i]), true)*eps_/100.0, true);
            if((result[i].size() == (maxVertices_)) && (fabs(contourArea(result[i]))>500))
            {
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
                    Scalar colour = Scalar(128,255,255);
                    polylines(imgLines, result[i], true, colour, 2);
                    circle(imgLines, mc[i], 4, colour, -1, 8, 0);
                    if (mc[i].x >= 0 && mc[i].y >= 0 && Last_[i].x >= 0 && Last_[i].y >= 0)
                    {
                        line(imgLinesLines, mc[i], Last_[i], colour, 2);
                    }
                    Last_[i] = mc[i];
                }
            }

        }
        imgLines = imgFrame_ + imgLinesLines + imgLines; //******************************
        return result;
    }

    void trackObject(Mat imgThresh)
    {
        vector<vector<Point> >contours;
        vector<Vec4i> hierarchy;

        findContours(imgThresh, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

        //vector<vector<Point> > ConvexHullPoints = contoursConvexHull(contours);
        vector<vector<Point> > approxPolyPoints = contoursApproxPoly(contours);

        //polylines( imgLines, ConvexHullPoints, true, Scalar(255,0,0), 1);
        polylines( imgLines, approxPolyPoints, true, Scalar(0,0,255), 1);

        return;
    }



    void imageEdgeTracker(cv_bridge::CvImagePtr cv_ptr)
    {
        Mat frame(cv_ptr->image);

        Mat imgThresholded;
        Mat edges;

        imgLines = Mat::zeros(cv_ptr->image.size(),CV_8UC3);

        cvtColor(frame, imgThresholded, COLOR_BGR2GRAY); //Convert the captured frame from BGR to GRAY
        imgFrame_ = frame.clone();


        if(gaussianScalar_ > 0 )
            GaussianBlur(imgThresholded, imgThresholded, cv::Size(0,0), gaussianScalar_/1000.0, gaussianScalar_/1000.0);

        Canny(imgThresholded, edges, edgeScalar_, edgeScalar_*2, 3, true);
        dilate(edges, edges, getStructuringElement(MORPH_RECT, Size(5,1)), Point(-1, -1));
        dilate(edges, edges, getStructuringElement(MORPH_ELLIPSE, Size(5,5)), Point(-1, -1));
        if(initFlag)
        {
            edgeLast_ = edges.clone();
            initFlag = false;
        }
        addWeighted(edges, 0.8, edgeLast_, 0.2, 0.2, edges);
        edgeLast_ = edges.clone();
        trackObject(edges);

        imshow("Edges", edges);
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
