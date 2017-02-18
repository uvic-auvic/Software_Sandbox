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
        Mat imgThresholded;
        Mat HSV;

        cvtColor(frame, HSV, CV_BGR2HSV);

        inRange(HSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV),imgThresholded);

        /* Display images/frames */
        imshow("Original",frame);
        imshow("Threshold", imgThresholded);

        /* Display Lower and Upper bounds for the scalar ranges in control */
        /* Create Lower and Upper bound Matrices from the HSV Scalar and convert to RGB colourspace (imshow doesn't handle HSV well) */
        Mat LowerBound, UpperBound;
        cvtColor(Mat(100,100, CV_8UC3).setTo(Scalar(iLowH, iLowS, iLowV)), LowerBound, CV_HSV2BGR);
        cvtColor(Mat(100,100, CV_8UC3).setTo(Scalar(iHighH, iHighS, iHighV)), UpperBound, CV_HSV2BGR);
        /* Create Mat combine with space to hold both Upper and Lower bound matrices */
        Mat combine(max(LowerBound.size().height, UpperBound.size().height), LowerBound.size().width+UpperBound.size().width, CV_8UC3);
        /* Fill left half of combine with Lower bound mat */
        Mat left_roi(combine, Rect(0,0,LowerBound.size().width, LowerBound.size().height));
        LowerBound.copyTo(left_roi);
        /* Fill right half of combine with Upper bound mat */
        Mat right_roi(combine, Rect(LowerBound.size().width, 0, UpperBound.size().width, UpperBound.size().height));
        UpperBound.copyTo(right_roi);
        imshow("Control", combine);

        getContours(frame, imgThresholded);

        imshow("Contours and Orientation",frame);

        /* Need to call waitKey for imshow to work */
        waitKey(3);
        return;
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
