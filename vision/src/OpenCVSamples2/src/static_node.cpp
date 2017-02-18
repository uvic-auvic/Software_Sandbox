#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>

using namespace cv;
RNG rng(12345);

class ImageConverter
{
    int lastX1_, lastY1_, lastX2_, lastY2_;
    int gaussianScalar_, edgeScalar_, edgeScalarUpper_, thresholdLowerScalar_, eps_;
    int maxAngle_, minAngle_, maxVertices_;

    Mat imgLines, edgeLast_;
    bool initFlag;


public:
    ImageConverter()
    {
        namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
        gaussianScalar_ = 0;
        thresholdLowerScalar_ = 100;
        edgeScalar_ = 100;
        edgeScalarUpper_ = 100;
        eps_ = 5;
        maxAngle_ = 180;
        minAngle_ = 0;
        maxVertices_ = 3;
        createTrackbar("Gaussian Filter", "Control", &gaussianScalar_, 4000);
        createTrackbar("Edge Threshold", "Control", &edgeScalar_, 255);
        createTrackbar("Epsilon", "Control", &eps_, 100);
        createTrackbar("Max Angle", "Control", &maxAngle_, 180);
        createTrackbar("Number of edges", "Control", &maxVertices_, 20);

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
            }

        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        //this->imageEdgeTracker(cv_ptr);

        int c = cv::waitKey(3);
        if((char)c == 27) ros::shutdown();

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

    float angle( Point pt1, Point pt2)
    {
        float dx1 = pt1.x;
        float dy1 = pt1.x;
        float dx2 = pt2.x;
        float dy2 = pt2.y;
        return (dx1*dx2 + dy1 * dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);

    }

    vector<vector<Point> > contoursConvexHull(vector<vector<Point> > contours)
    {
        vector<vector<Point> > result(contours.size());

        for (int i = 0; i < contours.size(); i++)
        {
            convexHull(Mat(contours[i]), result[i], true);
            if(fabs(contourArea(result[i]))>100 && isContourConvex(Mat(result[i])))
            {
                float maxCosine = minAngle_*M_PI/180;
                float minCosine = maxAngle_*M_PI/180;
                for(int j = 1; j < result[i].size(); j++)
                {
                    float cosine = fabs(angle(result[i][j], result[i][j-1]));
                    maxCosine = MAX(maxCosine, cosine);
                    minCosine = MIN(minCosine, cosine);
                }
                if(maxCosine < maxAngle_*M_PI/180 && minCosine > minAngle_*M_PI/180)
                    polylines(imgLines, result[i], true, Scalar(255,128,255), 2);
            }
        }
        return result;
    }

    vector<vector<Point> > contoursApproxPoly(vector<vector<Point> > contours)
    {
        vector<vector<Point> > result(contours.size());

        for (int i = 0; i < contours.size(); i++)
        {
            approxPolyDP(Mat(contours[i]), result[i], arcLength(Mat(contours[i]), true)*eps_/100.0, true);
            if((result[i].size() == (maxVertices_)) && (fabs(contourArea(result[i]))>500))
            {
                float maxCosine = 0;
                float minCosine = M_PI;
                for(int j = 1; j < maxVertices_ + 1 ; j++)
                {
                    float cosine;
                    if (j == 1)
                        cosine = fabs(angle(result[i][result[i].size()-1], result[i][1], result[i][0]));
                    else
                        cosine = fabs(angle(result[i][j%maxVertices_], result[i][j-2], result[i][j-1]));
                    std::stringstream ss;
                    ss << (j-1) << i;
                    maxCosine = MAX(maxCosine, cosine);
                    minCosine = MIN(minCosine, cosine);
                    std::stringstream SS;
                    SS << i << ":" << j-1 << ": " << cosine;
                    ROS_INFO("%s", SS.str().c_str());
                    putText(imgLines, ss.str(), Point(result[i][j-1].x + i, result[i][j-1].y), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,255));
                }
                std::stringstream SS;
                SS << "Max:" << maxCosine*180/M_PI << " Min:" << minCosine*180/M_PI;
                ROS_INFO("%s", SS.str().c_str());
                if(maxCosine < maxAngle_*M_PI/180)
                    polylines(imgLines, result[i], true, Scalar(128,255,255), 2);
            }

        }
        return result;
    }

    void trackObject(Mat imgThresh)
    {
        vector<vector<Point> >contours;
        vector<Vec4i> hierarchy;

        findContours(imgThresh, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

        vector<vector<Point> > approxPolyPoints = contoursApproxPoly(contours);

        polylines( imgLines, approxPolyPoints, true, Scalar(0,0,255), 1);

        return;
    }



    void imageEdgeTracker(Mat frame)
    {
        //Mat frame(cv_ptr->image);

        Mat imgThresholded;
        Mat edges;



        cvtColor(frame, imgThresholded, COLOR_BGR2GRAY); //Convert the captured frame from BGR to GRAY
        imgLines = frame.clone();


        if(gaussianScalar_ > 0 )
            GaussianBlur(imgThresholded, imgThresholded, cv::Size(0,0), gaussianScalar_/1000.0, gaussianScalar_/1000.0);

        Canny(imgThresholded, edges, edgeScalar_, edgeScalar_*2, 3, true);
        dilate(edges, edges, Mat());

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
    Mat image = imread("/home/ubuntu/ROV02/src/opencvtest/include/opencvtest/square.png");
    ic.imageEdgeTracker(image);
    while(1)
    {
        int c = cv::waitKey(3);
        if((char)c == 27)
            break;
    }
    //ros::spin();
    return 0;
}
