#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "thrusters/thrusterValues.h"

namespace gazebo
{
	class Joystick : public ModelPlugin
	{
		private: ros::NodeHandle nh_;
		
		private: int delay;
		
		private: math::Vector3 forceOld_, forceCurrent_, torqueOld_, torqueCurrent_;
		
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
			forceOld_ = math::Vector3(0,0,0);
			forceCurrent_ = math::Vector3(0,0,0);
			torqueOld_ = math::Vector3(0,0,0);
			torqueCurrent_ = math::Vector3(0,0,0);
			
            joy_sub_ = nh_.subscribe("/GUI/thrusterValues",10,&Joystick::joystickCb,this);
			
		  // Store the pointer to the model
		  this->model = _parent;
			
		  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			  boost::bind(&Joystick::OnUpdate, this, _1));
		}

		// Called by the world update start event
		public: void OnUpdate(const common::UpdateInfo & /*_info*/)
		{
			if(((++delay)%10) == 0)
			{
				physics::LinkPtr body;
				body = this->model->GetLink("body");
				body->AddForce(forceCurrent_);
				body->AddTorque(torqueCurrent_);
				/*Damping Force -- No longer provided in Joystick Plugin, now is a separate plugin damping_plugin*/
				//forceOld_ = body->GetRelativeLinearVel();
				
				//torqueOld_ = body->GetRelativeAngularVel();
				//body->AddRelativeForce(-forceOld_*500);
				//body->AddRelativeTorque(-torqueOld_*500);
				if(delay ==1500)
				{
					delay = 0;
				}
				
			}
		}
		
		
		
		// Called by the world update start event
        public: void joystickCb(thrusters::thrusterValues thrustValues)
		{
			
            math::Pose worldPose = this->model->GetWorldPose();
            math::Quaternion angle = worldPose.rot;
			
            /*std::cout << thrustValues << std::endl;			
            int scale = 10;
            this->model->SetLinearVel(angle.RotateVector(math::Vector3(scale*thrustValues.forwardThruster,0,0)));
            this->model->SetAngularVel(angle.RotateVector(math::Vector3(scale*thrustValues.rollThruster, scale*thrustValues.pitchThruster, scale*thrustValues.yawThruster)));
			*/
			physics::LinkPtr body;
			body = this->model->GetLink("body");
			int scale = 300;
			if(thrustValues.ascent == "Ascend")
			{
				forceCurrent_ = angle.RotateVector(math::Vector3(scale*thrustValues.forwardThruster, 0, 0.4*scale));
			} 
			else if (thrustValues.ascent == "Descend") 
			{
				forceCurrent_ = angle.RotateVector(math::Vector3(scale*thrustValues.forwardThruster, 0, -0.3*scale));
			}
			else
			{
				forceCurrent_ = angle.RotateVector(math::Vector3(scale*thrustValues.forwardThruster, 0, 0));
			}

			torqueCurrent_ = angle.RotateVector(math::Vector3(scale*thrustValues.rollThruster, scale*thrustValues.pitchThruster, scale*thrustValues.yawThruster));
		}

		
		
		
		
		// Pointer to the mode
		private: physics::ModelPtr model;

		// Pointer to the update event connection
		private: event::ConnectionPtr updateConnection;
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(Joystick);
}
