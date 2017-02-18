/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include <QtGui>
#include <QCheckBox>
#include <QMessageBox>
#include <iostream>
#include "../include/redgui/main_window.hpp"
#include <stdio.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace redgui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{

	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*
     * Video feed
     */
    QObject::connect(&qnode, SIGNAL(processedImage(QImage)), this, SLOT(updatePlayerUI(QImage)));

    /*
     * Thruster Sensitivities
     */
    QObject::connect(ui.forwardFactor, SIGNAL(valueChanged(int)), this, SLOT(updateForwardSensitivity(int)));
    QObject::connect(ui.pitchFactor, SIGNAL(valueChanged(int)), this, SLOT(updatePitchSensitivity(int)));
    QObject::connect(ui.rollFactor, SIGNAL(valueChanged(int)), this, SLOT(updateRollSensitivity(int)));
    QObject::connect(ui.yawFactor, SIGNAL(valueChanged(int)), this, SLOT(updateYawSensitivity(int)));
    QObject::connect(ui.ascentFactor, SIGNAL(valueChanged(int)), this, SLOT(updateAscentSensitivity(int)));

    /*
     * Thruster Values
     */
    QObject::connect(&qnode, SIGNAL(thrusterForwardSignal(float)), this, SLOT(updateForwardThrusterBar(float)));
    QObject::connect(&qnode, SIGNAL(thrusterPitchSignal(float)), this, SLOT(updatePitchThrusterBar(float)));
    QObject::connect(&qnode, SIGNAL(thrusterRollSignal(float)), this, SLOT(updateRollThrusterBar(float)));
    QObject::connect(&qnode, SIGNAL(thrusterYawSignal(float)), this, SLOT(updateYawThrusterBar(float)));
    QObject::connect(&qnode, SIGNAL(thrusterAscentSignal(float)), this, SLOT(updateAscentThrusterBar(float)));
    /*
     * Thruster Lockout - autofilled
     */
    //QObject::connect(ui.lightsPushButton, SIGNAL(toggled(bool)), this, SLOT(on_lightsPushButton_toggled(bool)));
    /*
     * Thruster control mode (AUV or ROV)
     */
    QObject::connect(ui.ROVtoAUVPushButton, SIGNAL(released()), this, SLOT(update_ROVtoAUVPushButton()));
    /*
     * GUI Cam Video Record
     */
    QObject::connect(ui.videoRecordPushButton, SIGNAL(released()), this, SLOT(updateVideoRecordPushButton()));
    /*
     *  Temperature Values
     */
    QObject::connect(&qnode, SIGNAL(UCDieTemperatureSignal(float)), this, SLOT(updateUCDieTemperatureLCD(float)));
    QObject::connect(&qnode, SIGNAL(OILTemperatureSignal(float)), this, SLOT(updateOILTemperatureLCD(float)));

    /*
     * Trim stuff
     */
    QObject::connect(ui.forwardTrimHorizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(updateForwardTrimSliderText(int)));
    QObject::connect(ui.forwardTrimLineEdit, SIGNAL(textChanged(QString)), this, SLOT(updateForwardTrimSliderPos(QString)));

    QObject::connect(ui.pitchTrimHorizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(updatePitchTrimSliderText(int)));
    QObject::connect(ui.pitchTrimLineEdit, SIGNAL(textChanged(QString)), this, SLOT(updatePitchTrimSliderPos(QString)));

    QObject::connect(ui.yawTrimHorizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(updateYawTrimSliderText(int)));
    QObject::connect(ui.yawTrimLineEdit, SIGNAL(textChanged(QString)), this, SLOT(updateYawTrimSliderPos(QString)));

    QObject::connect(ui.rollTrimHorizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(updateRollTrimSliderText(int)));
    QObject::connect(ui.rollTrimLineEdit, SIGNAL(textChanged(QString)), this, SLOT(updateRollTrimSliderPos(QString)));
    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
            //ui.line_edit_topic->setReadOnly(true);
            ui.ControlsGroupBox->setEnabled(true);

            ui.throttleLockoutCheckBox->setEnabled(true);
            ui.joystick_tab->setEnabled(true);
            qnode.throttleLockoutChange(true);
            ui.forwardFactor->setValue(qnode.sensitivityData("forwardSensitivity"));
            ui.pitchFactor->setValue(qnode.sensitivityData("pitchSensitivity"));
            ui.rollFactor->setValue(qnode.sensitivityData("rollSensitivity"));
            ui.yawFactor->setValue(qnode.sensitivityData("yawSensitivity"));
            ui.ascentFactor->setValue(qnode.sensitivityData("ascentSensitivity"));
            qnode.sensitivityPublish();

            if(qnode.whichControlMode() == 2)
                ui.ROVtoAUVPushButton->setText("AUV mode");
            /* XXXXX */
            ui.videoRecordPushButton->setEnabled(false); /* XXXXXX
                                                          * Disabled for demonstration days so people don't save gigs of video data and overload
           /* XXXXX                                       * the harddrive. Comment out for normal use to give the option to record video.
                                                          * XXXXXX*/
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}
/*
 * Update Camera Feed
 */
void MainWindow::updatePlayerUI(QImage img){
    if(!img.isNull()){
        ui.camera_feed->setAlignment(Qt::AlignCenter);
        //ui.camera_feed->setPixmap(QPixmap::fromImage(img).scaled(img.size().width(), img.size().height(), Qt::KeepAspectRatio, Qt::FastTransformation));
        ui.camera_feed->setPixmap(QPixmap::fromImage(img).scaled(ui.camera_feed->geometry().width(), ui.camera_feed->geometry().height(), Qt::KeepAspectRatio, Qt::FastTransformation));
    }
}
/*Update thruster sensitivities
 */
void MainWindow::updateForwardSensitivity(int value)
{
    ui.forwardFactor->setValue(value);
    qnode.forwardSensitivity(value);
}

void MainWindow::updatePitchSensitivity(int value)
{
    ui.pitchFactor->setValue(value);
    qnode.pitchSensitivity(value);
}

void MainWindow::updateRollSensitivity(int value)
{
    ui.rollFactor->setValue(value);
    qnode.rollSensitivity(value);
}

void MainWindow::updateYawSensitivity(int value)
{
    ui.yawFactor->setValue(value);
    qnode.yawSensitivity(value);
}

void MainWindow::updateAscentSensitivity(int value)
{
    ui.ascentFactor->setValue(value);
    qnode.ascentSensitivity(value);
}
/* Update Thruster Bars
 */
void MainWindow::updateForwardThrusterBar(float thrustValue)
{
    int value = (thrustValue);
    ui.forwardThrusterBar->setValue(value);
}

void MainWindow::updatePitchThrusterBar(float thrustValue)
{
    int value = (thrustValue);
    ui.pitchThrusterBar->setValue(value);
}

void MainWindow::updateRollThrusterBar(float thrustValue)
{
    int value = (thrustValue);
    ui.rollThrusterBar->setValue(value);
}

void MainWindow::updateYawThrusterBar(float thrustValue)
{
    int value = (thrustValue);
    ui.yawThrusterBar->setValue(value);
}

void MainWindow::updateAscentThrusterBar(float thrustValue)
{
    int value = (thrustValue);
    ui.ascentThrusterBar->setValue(value);
}

/*
 * Update Control Mode (AUV or ROV)
 */

void MainWindow::update_ROVtoAUVPushButton()
{
    if(qnode.whichControlMode() == 1)
    {
        /* Currently in ROV mode and switching to AUV mode */
        ui.ROVtoAUVPushButton->setText("AUV mode");
        qnode.updateControlMode("AUV");
    }
    else if(qnode.whichControlMode() == 2)
    {
        /* Currently in AUV mode and switching to ROV mode */
        ui.ROVtoAUVPushButton->setText("ROV mode");
        qnode.updateControlMode("ROV");
    }
    else
    {
       ui.ROVtoAUVPushButton->setText("ROV mode");
       qnode.updateControlMode("ROV");
    }
}

/*
 * Update video record mode (Record or Stop)
 */
void MainWindow::updateVideoRecordPushButton()
{
    /* 0 is video recording. 1 or other is not video recording. */
    if(qnode.whichVideoRecordMode() == 0)
    {
        /* Video was recording before button pressed, switching to stop recording and give option to record video again. */
        ui.videoRecordPushButton->setText("Record Video");
        qnode.updateVideoRecordMode(1);
    }
    else
    {
       /* Video was not recording and switching to start recording */
       ui.videoRecordPushButton->setText("Stop Recording");
       qnode.updateVideoRecordMode(0);
    }
}


/*
 * Update Throttle Lockout
 */
void MainWindow::on_throttleLockoutCheckBox_stateChanged(int state)
{
    bool enabled;
    if(state == 0){
        QString unlocked = "QProgressBar::chunk {background:  rgb(48, 181, 8);} QProgressBar {text-align: center;}";
        ui.ascentThrusterBar->setStyleSheet(unlocked);
        ui.yawThrusterBar->setStyleSheet(unlocked);
        ui.pitchThrusterBar->setStyleSheet(unlocked);
        ui.forwardThrusterBar->setStyleSheet(unlocked);
        ui.rollThrusterBar->setStyleSheet(unlocked);
        enabled = false;
    }
    else
    {
        QString locked =  "QProgressBar::chunk {background: rgb(172, 172, 172);} QProgressBar {text-align: center;}";
        ui.ascentThrusterBar->setStyleSheet(locked);
        ui.yawThrusterBar->setStyleSheet(locked);
        ui.pitchThrusterBar->setStyleSheet(locked);
        ui.forwardThrusterBar->setStyleSheet(locked);
        ui.rollThrusterBar->setStyleSheet(locked);
        enabled = true;
    }
    qnode.throttleLockoutChange(enabled);
}

/*
 *  Update light state
 */
void MainWindow::on_lightsCheckBox_stateChanged(int state)
{
    bool enabled;
    if(state == 0){
        enabled = false;
    }
    else
    {
        enabled = true;
    }
    qnode.lightChange(enabled);
}
/* invert thruster axis */
void MainWindow::on_forwardInvertCheckBox_stateChanged(int state)
{
    bool enabled;
    if(state == 0){
        enabled = false;
    }
    else
    {
        enabled = true;
    }
    qnode.forwardInvertChange(enabled);
}

void MainWindow::on_pitchInvertCheckBox_stateChanged(int state)
{
    bool enabled;
    if(state == 0){
        enabled = false;
    }
    else
    {
        enabled = true;
    }
    qnode.pitchInvertChange(enabled);
}

void MainWindow::on_rollInvertCheckBox_stateChanged(int state)
{
    bool enabled;
    if(state == 0){
        enabled = false;
    }
    else
    {
        enabled = true;
    }
    qnode.rollInvertChange(enabled);
}

void MainWindow::on_yawInvertCheckBox_stateChanged(int state)
{
    bool enabled;
    if(state == 0){
        enabled = false;
    }
    else
    {
        enabled = true;
    }
    qnode.yawInvertChange(enabled);
}

/* Update Temperature Sensors displays
 */
void MainWindow::updateUCDieTemperatureLCD(float value)
{
    value = value*100;
    ui.UCDieTemperatureLCD->display(value);
    if(value > 90)
    {
        // Critical temperature reached!! Display RED
        ui.UCDieTemperatureLCD->setStyleSheet("QLCDNumber{ background-color: rgb(191,0,0)}");
    }
    else if(value > 70)
    {
        // Warning temperature reached!! Display ORANGE
        ui.UCDieTemperatureLCD->setStyleSheet("QLCDNumber{ background-color: rgb(215,165,0)}");
    }
    else if(value > 10)
    {
        // Running cool~ Display BLUE
        ui.UCDieTemperatureLCD->setStyleSheet("QLCDNumber{ background-color: rgb(24,0,176)}");
    }
    else
    {
        // Running cold. Display BLACK
        ui.UCDieTemperatureLCD->setStyleSheet("QLCDNumber{ background-color: black}");
    }
}

void MainWindow::updateOILTemperatureLCD(float value)
{
    value = value*100;
    ui.oilTemperatureLCD->display(value);
    if(value > 90)
    {
        // Critical temperature reached!! Display RED
        ui.oilTemperatureLCD->setStyleSheet("QLCDNumber{ background-color: rgb(191,0,0)}");
    }
    else if(value > 70)
    {
        // Warning temperature reached!! Display ORANGE
        ui.oilTemperatureLCD->setStyleSheet("QLCDNumber{ background-color: rgb(215,165,0)}");
    }
    else if(value > 10)
    {
        // Running cool~ Display BLUE
        ui.oilTemperatureLCD->setStyleSheet("QLCDNumber{ background-color: rgb(24,0,176)}");
    }
    else
    {
        // Running cold. Display BLACK
        ui.oilTemperatureLCD->setStyleSheet("QLCDNumber{ background-color: black}");
    }
}

void MainWindow::updateForwardTrimSliderPos(QString qs)
{
    QRegExp re("[-+]?[0-9]*");
    if(re.exactMatch(qs))
    {
        int value;
        bool ok;
        value = qs.toInt(&ok, 10);
        if(ok && value <= ui.forwardTrimHorizontalSlider->maximum() && value >= ui.forwardTrimHorizontalSlider->minimum())
        {
            ui.forwardTrimHorizontalSlider->setValue(value);
        }
        else
        {
            ROS_WARN("Only numerical values between %d and %d are acceptable!", ui.forwardTrimHorizontalSlider->minimum(), ui.forwardTrimHorizontalSlider->maximum());
            ui.forwardTrimLineEdit->setText((QString("%1").arg(ui.forwardTrimHorizontalSlider->value())));
        }
    }
    else
    {
        ROS_WARN("Only numerical values between %d and %d are acceptable!", ui.forwardTrimHorizontalSlider->minimum(), ui.forwardTrimHorizontalSlider->maximum());
        ui.forwardTrimLineEdit->setText((QString("%1").arg(ui.forwardTrimHorizontalSlider->value())));
    }
    return;
}

void MainWindow::updateForwardTrimSliderText(int value)
{
    ui.forwardTrimLineEdit->setText(QString("%1").arg(value));
    return;
}

void MainWindow::updatePitchTrimSliderPos(QString qs)
{
    QRegExp re("[-+]?[0-9]*");
    if(re.exactMatch(qs))
    {
        int value;
        bool ok;
        value = qs.toInt(&ok, 10);
        if(ok && value <= ui.pitchTrimHorizontalSlider->maximum() && value >= ui.pitchTrimHorizontalSlider->minimum())
        {
            ui.pitchTrimHorizontalSlider->setValue(value);
        }
        else
        {
            ROS_WARN("Only numerical values between %d and %d are acceptable!", ui.pitchTrimHorizontalSlider->minimum(), ui.pitchTrimHorizontalSlider->maximum());
            ui.pitchTrimLineEdit->setText((QString("%1").arg(ui.pitchTrimHorizontalSlider->value())));
        }
    }
    else
    {
        ROS_WARN("Only numerical values between %d and %d are acceptable!", ui.pitchTrimHorizontalSlider->minimum(), ui.pitchTrimHorizontalSlider->maximum());
        ui.pitchTrimLineEdit->setText((QString("%1").arg(ui.pitchTrimHorizontalSlider->value())));
    }
    return;
}

void MainWindow::updatePitchTrimSliderText(int value)
{
    ui.pitchTrimLineEdit->setText(QString("%1").arg(value));
    return;
}
void MainWindow::updateYawTrimSliderPos(QString qs)
{
    QRegExp re("[-+]?[0-9]*");
    if(re.exactMatch(qs))
    {
        int value;
        bool ok;
        value = qs.toInt(&ok, 10);
        if(ok && value <= ui.yawTrimHorizontalSlider->maximum() && value >= ui.yawTrimHorizontalSlider->minimum())
        {
            ui.yawTrimHorizontalSlider->setValue(value);
        }
        else
        {
            ROS_WARN("Only numerical values between %d and %d are acceptable!", ui.yawTrimHorizontalSlider->minimum(), ui.yawTrimHorizontalSlider->maximum());
            ui.yawTrimLineEdit->setText((QString("%1").arg(ui.yawTrimHorizontalSlider->value())));
        }
    }
    else
    {
        ROS_WARN("Only numerical values between %d and %d are acceptable!", ui.yawTrimHorizontalSlider->minimum(), ui.yawTrimHorizontalSlider->maximum());
        ui.yawTrimLineEdit->setText((QString("%1").arg(ui.yawTrimHorizontalSlider->value())));
    }
    return;
}

void MainWindow::updateYawTrimSliderText(int value)
{
    ui.yawTrimLineEdit->setText(QString("%1").arg(value));
    return;
}
void MainWindow::updateRollTrimSliderPos(QString qs)
{
    QRegExp re("[-+]?[0-9]*");
    if(re.exactMatch(qs))
    {
        int value;
        bool ok;
        value = qs.toInt(&ok, 10);
        if(ok && value <= ui.rollTrimHorizontalSlider->maximum() && value >= ui.rollTrimHorizontalSlider->minimum())
        {
            ui.rollTrimHorizontalSlider->setValue(value);
        }
        else
        {
            ROS_WARN("Only numerical values between %d and %d are acceptable!", ui.rollTrimHorizontalSlider->minimum(), ui.rollTrimHorizontalSlider->maximum());
            ui.rollTrimLineEdit->setText((QString("%1").arg(ui.rollTrimHorizontalSlider->value())));
        }
    }
    else
    {
        ROS_WARN("Only numerical values between %d and %d are acceptable!", ui.rollTrimHorizontalSlider->minimum(), ui.rollTrimHorizontalSlider->maximum());
        ui.rollTrimLineEdit->setText((QString("%1").arg(ui.rollTrimHorizontalSlider->value())));
    }
    return;
}

void MainWindow::updateRollTrimSliderText(int value)
{
    ui.rollTrimLineEdit->setText(QString("%1").arg(value));
    return;
}
/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "redgui");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "redgui");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace redgui

