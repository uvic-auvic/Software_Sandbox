/**
 * @file /include/redgui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef redgui_QNODE_HPP_
#define redgui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QWidget>
#include <QThread>
#include <QStringListModel>
#include <QLabel>
#include <QApplication>
#include <QStatusBar>
#include <QImage>
#include <QKeyEvent>
#include <QMenu>
#include <QMainWindow>
#include <QDebug>
#include <QFileDialog>
#include <QMenuBar>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "thrusters/Sensitivity.h"
#include "thrusters/thrusterValues.h"
#include "thrusters/thrusterTemperatures.h"
#include "std_msgs/Int32.h"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace redgui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void chatterCb(const std_msgs::String::ConstPtr& msg);
    void forwardSensitivity(int value);
    void pitchSensitivity(int value);
    void rollSensitivity(int value);
    void yawSensitivity(int value);
    void ascentSensitivity(int value);
    void sensitivityPublish();
    void thrusterValueCb(const thrusters::thrusterValues thrustValues);
    void lightChange(bool lightState);
    void throttleLockoutChange(bool lockoutState);
    void temperatureCb(const thrusters::thrusterTemperatures temperatures);
    int sensitivityData(std::string whichData);
    void forwardInvertChange(bool invertState);
    void rollInvertChange(bool invertState);
    void pitchInvertChange(bool invertState);
    void yawInvertChange(bool invertState);
    void updateControlMode(std::string controlMode);
    void updateVideoRecordMode(int value);
    int whichControlMode();
    int whichVideoRecordMode();

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void processedImage(const QImage &image);
    void valueChanged(float value);
    void thrusterForwardSignal(float thrustValue);
    void thrusterPitchSignal(float thrustValue);
    void thrusterRollSignal(float thrustValue);
    void thrusterYawSignal(float thrustValue);
    void UCDieTemperatureSignal(float value);
    void OILTemperatureSignal(float value);
    void thrusterAscentSignal(float ascentCommand);

private:
	int init_argc;
    int _listViewThreshold;
	char** init_argv;
	ros::Publisher chatter_publisher;
    ros::Publisher thrusterSensitivity_publisher;
    ros::Publisher throttleLockout_publisher;
    ros::Publisher lightState_publisher;
    ros::Publisher GUI_ROV_to_AUV_pub_;
    ros::Publisher GUI_record_mode_pub_;
    ros::Subscriber thrusterValue_subscriber;
    ros::Subscriber thrusterTemperature_subscriber;
    ros::Subscriber chatter_subscriber;
    image_transport::Subscriber camera_subscriber;
    QStringListModel logging_model;
    QImage img;
    cv::Mat RGBframe;
    thrusters::Sensitivity sensitivityData_;
    std_msgs::Int16 ROVtoAUVcontrolMode, videoRecordMode;
    bool forwardInvert_, rollInvert_, pitchInvert_, yawInvert_;

};

}  // namespace redgui

#endif /* redgui_QNODE_HPP_ */
