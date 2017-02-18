#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <math.h>
#include <turtlesim/Pose.h>

#define buoy_cascade "/home/googly/catkin_ws/src/opencvtest/include/opencvtest/buoy_cascade.xml"
#define BUOY_WIDTH 200 //Actual width of object mm
#define BUOY_HEIGHT 260
//#define FOCAL 100*600/BUOY_WIDTH // FOCAL Length for 640x480  F = PxD/W
#define FOCAL 130*600/BUOY_HEIGHT // FOCAL Length for 640x480  F = PxD/H

/*
 * A quickly made program to open up a video file and repeat it while applying a cascade classifier over it to
 * detect objects. Used to quickly test the quality of the cascade classifier.
 *
 */

using namespace cv;
RNG rng(12345);

class ImageConverter
{
    ros::NodeHandle nh_;
    ros::Publisher pos_pub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    Mat imgFrame_;
    CascadeClassifier classifier;

    VideoCapture video;
    int frames;
    int height_, ScaleFactor_, minNeighbours_;
    double fps;
    time_t timeLast_, timeNow_;

public:

    ImageConverter()
        : it_ (nh_)
    {
        pos_pub_ = nh_.advertise<turtlesim::Pose>("camera1/pose", 1);
        height_ = 480;
        ScaleFactor_ = 40;
        minNeighbours_ = 3;

        video.open((std::string(getenv("HOME")) + "/catkin_ws/src/opencvtest/include/opencvtest/buoy3.avi"));
        if(!video.isOpened())
        {
            std::cout << "Video failed to open " << (std::string(getenv("HOME")) + "/catkin_ws/src/opencvtest/include/opencvtest/buoy.MP4") << std::endl;
            return;
        }

        if(!classifier.load( buoy_cascade))
        {
            std::cout << "Error loading Face cascade." << std::endl;
        }

        namedWindow("Control");
        createTrackbar("Scale factor", "Control", &ScaleFactor_, 100);
        createTrackbar("minNeighbours", "Control", &minNeighbours_, 10);

        double start = video.get(CV_CAP_PROP_POS_FRAMES);

        for(int i = 0; ; i++)
        {
            Mat frame;
            video.read(frame);
            if(frame.empty())
            {
                /* Frame is at the end of the video, let's repeat! Again! Again!~ */
                std::cout<<"End of video! Repeating now. Again! Again!"<<std::endl;
                video.set(CV_CAP_PROP_POS_FRAMES, start);
                i = 0;
                video.read(frame);
            }
            resize(frame,frame, Size(640,480));
            imageEdgeTracker(frame);
            std::cout<<"Frame:"<<i<<std::endl;
            if(waitKey(10) != -1)
                break;
        }

    }

    ~ImageConverter()
    {
        cv::destroyAllWindows();
    }

    void cascadeTracker(Mat imgThresh)
    {
        vector<Rect> faces;
        turtlesim::Pose msg;
        double scaleFactor = 1+ScaleFactor_/100.0;
        if(scaleFactor <= 1.05)
            scaleFactor = 1.05;
        classifier.detectMultiScale(imgThresh, faces, scaleFactor, minNeighbours_, 0|CV_HAAR_SCALE_IMAGE, Size(5,5), Size(imgThresh.cols/2, imgThresh.rows/2));

        for(int i = 0; i < faces.size(); i++)
        {
            //For all found objects, draw an ellipse over it and get the coordinates x,y,z
            Point center( (faces[i].x + faces[i].width*0.5), (faces[i].y + faces[i].height*0.5) );
            ellipse ( imgFrame_, center, Size(faces[i].height*0.5, faces[i].width*0.5), 0, 0, 360, Scalar(255,128,255), 4, 8, 0);
            float focal = FOCAL; // *(height_)/(1200);
            float hdistance = ((BUOY_HEIGHT * focal) / (faces[i].height) ) / 10.0;
            //float wdistance = ((BUOY_WIDTH * focal) / (faces[i].width) ) / 10.0;

            float xCoord = (((center.x - imgFrame_.size().width/2) * hdistance) / focal );
            float yCoord = (((imgFrame_.size().height/2 - center.y) * hdistance) / focal );

            std::stringstream textss;
            textss <<"("<<" "<< round(xCoord) << "cm," << round(yCoord) << "cm," << round(hdistance) <<"cm)";
            putText(imgFrame_, textss.str().c_str(), center, FONT_HERSHEY_PLAIN, 1.2, Scalar(0,0,255), 2);

            msg.x = xCoord / 100.0;
            msg.y = yCoord / 100.0;
            msg.theta = hdistance / 100.0;

            pos_pub_.publish(msg);

        }

        return;
    }

    void imageEdgeTracker(Mat frame)
    {

        Mat imgThresholded;

        imgFrame_ = frame.clone();
        //Convert to gray scale for better object detection.
        cvtColor(frame, imgThresholded, CV_BGR2GRAY);
        //Equalize histrograme
        equalizeHist( imgThresholded, imgThresholded);
        //Check for object using the cascade classifier.
        cascadeTracker(imgThresholded);
        // Draw crosshairs over the image and put a (0,0) over the origin.
        putText(imgFrame_, "(0,0)", Point(imgFrame_.size().width/2, imgFrame_.size().height/2), 1, FONT_HERSHEY_PLAIN,Scalar(0,0,255));
        line(imgFrame_,Point(imgFrame_.size().width/2, 0), Point(imgFrame_.size().width/2, imgFrame_.size().height), Scalar(0,0,255));
        line(imgFrame_,Point(0, imgFrame_.size().height/2), Point(imgFrame_.size().width, imgFrame_.size().height/2), Scalar(0,0,255));
        //Show image with recognized shape/object.
        imshow("Frame", imgFrame_);
        waitKey(1);
        return;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "banana_test");
    ImageConverter ic;
    //ros::spin();
    return 0;
}

