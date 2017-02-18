/**
 * @file /include/redgui/main_window.hpp
 *
 * @brief Qt based gui for redgui.
 *
 * @date November 2010
 **/
#ifndef redgui_MAIN_WINDOW_H
#define redgui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace redgui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void updatePlayerUI(QImage img);
    void updateForwardSensitivity(int value);
    void updatePitchSensitivity(int value);
    void updateRollSensitivity(int value);
    void updateYawSensitivity(int value);
    void updateAscentSensitivity(int value);
    void updateForwardThrusterBar(float thrustValue);
    void updatePitchThrusterBar(float thrustValue);
    void updateRollThrusterBar(float thrustValue);
    void updateYawThrusterBar(float thrustValue);
    void updateAscentThrusterBar(float ascentCommand);
    void updateUCDieTemperatureLCD(float value);
    void updateOILTemperatureLCD(float value);
    void update_ROVtoAUVPushButton();
    void updateVideoRecordPushButton();
    void on_lightsCheckBox_stateChanged(int state);
    void on_throttleLockoutCheckBox_stateChanged(int state);
    void on_forwardInvertCheckBox_stateChanged(int state);
    void on_pitchInvertCheckBox_stateChanged(int state);
    void on_rollInvertCheckBox_stateChanged(int state);
    void on_yawInvertCheckBox_stateChanged(int state);

    void updateForwardTrimSliderText(int value);
    void updateForwardTrimSliderPos(QString qs);
    void updatePitchTrimSliderText(int value);
    void updatePitchTrimSliderPos(QString qs);
    void updateYawTrimSliderText(int value);
    void updateYawTrimSliderPos(QString qs);
    void updateRollTrimSliderText(int value);
    void updateRollTrimSliderPos(QString qs);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace redgui

//Q_DECLARE_METATYPE(thrusters::thrusterValues)
#endif // redgui_MAIN_WINDOW_H
