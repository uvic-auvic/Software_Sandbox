#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

namespace gazebo
{
	class Joystick : public ModelPlugin
	{
		private: ros::NodeHandle nh_;
		
		private: int delay;
		
		private: ros::Subscriber joy_sub_;
		
		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
		{
			delay = 0;
            // Make sure the ROS node for Gazebo has already been initialized
            if (!ros::isInitialized())
            {
              ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
              return;
            }

            ROS_INFO("Hello World!");
			
            joy_sub_ = nh_.subscribe("/GUI/thrusterValues",10,&Joystick::joystickCb,this);
			
		  // Store the pointer to the model
		  this->model = _parent;
			
		  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			  boost::bind(&Joystick::OnUpdate, this, _1));
		}

		// Called by the world update start event
		public: void OnUpdate(const common::UpdateInfo & /*_info*/)
		{
			if(++delay == 1500)
			{
				delay = 0;
				physics::LinkPtr propeller;
				propeller = this->model->GetLink("propeller");
				/* World Pose: X, Y, Z, Roll, Pitch, Yaw */
				std::cout << "World Pose: " << model->GetWorldPose() << std::endl;
                math::Quaternion angle = this->model->GetWorldPose().rot;
                math::Vector3 eulerAngle;
                eulerAngle = angle.GetAsEuler();
                std::cout << "eulerAngle: " << eulerAngle << std::endl;
			}
		}
		
		
		
		// Called by the world update start event
		public: void joystickCb(const std_msgs::String::ConstPtr& msg)
		{
			std::string token;
			char* msgString = strdup(msg->data.c_str());
			bool dirtyButton = false;
			bool dirtyAxis = false;
			int eventNumber_, eventValue_;
			
			token = strtok(msgString,";");
			if(token == "Button")
			{
				/* Joystick command is a button
				 */
				token = strtok(NULL,";");
				eventNumber_ = strtol(token.c_str(),0,10);
				token = strtok(NULL,";");
				eventValue_ = strtol(token.c_str(),0,10);
				dirtyButton = true;
			}
			else if(token == "Axis")
			{
				/* Joystick command is an Axis
				 */
				token = strtok(NULL,";");
				eventNumber_ = strtol(token.c_str(),0,10);
								if(eventNumber_ == 2)
				{
					token = strtok(NULL,";");
					eventValue_ = strtol(token.c_str(),0,10);
				}
				dirtyAxis = true;
			}

			else
			{
				/* Unexepected value. Should not occur.
				 */
				ROS_DEBUG("Unexpected joystick command %s", msg->data.c_str());
			}
			
			if(dirtyButton == true)
			{
				/* Joystick Button command was found, time to send to thrusters
				 */
				ROS_INFO("Button %d is %d", eventNumber_, eventValue_);
			}
			else if(dirtyAxis == true)
			{
				/* Joystick Axis command was found, time to send to thrusters
				 */
				ROS_INFO("Axis %d at %d", eventNumber_, eventValue_);
						 
				// Apply a small rotational velocity to the model.
				if(eventNumber_ == 2)
				{
					//physics::LinkPtr propeller;
					//propeller = this->model->GetLink("propeller");
					if (eventValue_ > 32000)
						eventValue_ = 32000;
					if(eventValue_ < -32000)
					   eventValue_ = -32000;

                    math::Pose worldPose = this->model->GetWorldPose();
                    math::Quaternion angle = worldPose.rot;
                    math::Vector3 eulerAngle;
                    eulerAngle = angle.GetAsEuler();
                    this->model->SetLinearVel(angle.RotateVector(math::Vector3(0,0,0.0001*eventValue_)));
				}
			}
		}

		
		
		
		
		// Pointer to the mode
		private: physics::ModelPtr model;

		// Pointer to the update event connection
		private: event::ConnectionPtr updateConnection;
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(Joystick);
}
